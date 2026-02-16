"""
Cat detector: returns best 'cat' bbox or None.
Stub: returns None. Replace with vilib/TFLite COCO (class 16 = cat).
"""
from typing import Optional, Tuple

# Stub: no camera
def get_cat_bbox(image=None, image_width: int = 640, image_height: int = 480) -> Optional[Tuple[float, float, float, float]]:
    """
    Returns (x, y, w, h) in pixels for best cat in image, or None.
    Stub: always returns None until vilib detector is wired.
    """
    return None
