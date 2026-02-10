import os


def _first_existing_path(paths):
    for p in paths:
        if p and os.path.isfile(p):
            return p
    return None


def resolve_model_path(model_variant, override_model_path=None):
    """
    Resolve the model weights path for a given variant.

    model_variant: "yolo26" or "yolov8"
    override_model_path: if set and exists, used directly.
    """
    if override_model_path:
        if os.path.isfile(override_model_path):
            return override_model_path
        raise FileNotFoundError(f"~model_path does not exist: {override_model_path}")

    # Defaults for this workspace (keep absolute paths for ROS nodes launched anywhere).
    # YOLO26 files in this repo:
    # - /home/tnlab/catkin_ws/src/yolo-ros/best_26.pt (your trained weights)
    # - /home/tnlab/catkin_ws/yolo26n-seg.pt (provided model file)
    defaults = {
        "yolo26": [
            "/home/tnlab/catkin_ws/src/yolo-ros/best_26.pt",
            "/home/tnlab/catkin_ws/yolo26n-seg.pt",
        ],
        "yolov8": [
            "/home/tnlab/catkin_ws/src/yolo-ros/best_v8.pt",
        ],
    }

    candidates = defaults.get(str(model_variant).strip().lower())
    if not candidates:
        raise ValueError(f"Unknown model_variant '{model_variant}'. Use 'yolo26' or 'yolov8'.")

    p = _first_existing_path(candidates)
    if not p:
        raise FileNotFoundError(
            f"No default weights found for model_variant='{model_variant}'. Tried: {candidates}"
        )
    return p


def load_ultralytics_yolo(model_path):
    """
    Load an Ultralytics YOLO model, with a helpful error message when the
    installed ultralytics is too old for the checkpoint (e.g. missing C3k2).
    """
    from ultralytics import YOLO
    import ultralytics

    try:
        return YOLO(model_path)
    except Exception as e:
        msg = str(e)
        # Common when trying to load newer-model checkpoints (YOLO26/YOLOv11-style)
        # with an older ultralytics install.
        if "C3k2" in msg or "Can't get attribute 'C3k2'" in msg:
            raise RuntimeError(
                "Failed to load model weights because this checkpoint requires a newer "
                "Ultralytics install (missing module 'C3k2').\n"
                f"- model_path: {model_path}\n"
                f"- ultralytics version: {getattr(ultralytics, '__version__', 'unknown')}\n"
                "Fix:\n"
                "  pip install -U ultralytics\n"
                "Then re-run the node."
            ) from e
        raise

