"""Depth camera visualization window, backed by Tkinter and Pillow."""
from __future__ import annotations

import numpy as np


class DepthWindowError(RuntimeError):
    """Raised when the depth window cannot be created or updated."""


class DepthWindow:
    """Shows depth images in a Tk window.

    Closing the window only stops the visualization. The depth sensor keeps
    running because rendering and publishing happen in the simulation loop.
    """

    def __init__(self, width: int, height: int, *, scale: int = 4,
                 max_range_m: float = 5.0, margin: int = 60,
                 always_on_top: bool = True,
                 title: str = 'A1 depth camera') -> None:
        if width <= 0 or height <= 0:
            raise ValueError('depth window size must be positive')
        if max_range_m <= 0.0:
            raise ValueError('max_range_m must be greater than zero')
        self._width = int(width)
        self._height = int(height)
        self._scale = max(1, int(scale))
        self._max_range = float(max_range_m)
        self._margin = max(0, int(margin))
        self._always_on_top = bool(always_on_top)
        self._title = title
        self._root = None
        self._label = None
        self._photo = None
        self._closed = False

    @property
    def window_size(self) -> tuple[int, int]:
        """On-screen size of the window, in pixels."""
        return self._width * self._scale, self._height * self._scale

    @property
    def closed(self) -> bool:
        return self._closed

    def show(self, depth: np.ndarray) -> None:
        """Draw one depth frame, raising ``DepthWindowError`` on failure."""
        if self._closed:
            return
        try:
            from PIL import ImageTk

            if self._root is None:
                self._create_window()
            self._photo = ImageTk.PhotoImage(self._to_image(depth))
            self._label.configure(image=self._photo)
            self._root.update_idletasks()
            self._root.update()
        except DepthWindowError:
            raise
        except Exception as error:  # tkinter/Pillow raise backend specific errors
            self.close()
            raise DepthWindowError(str(error)) from error

    def close(self) -> None:
        """Destroy the window and refuse further frames."""
        self._closed = True
        if self._root is not None:
            try:
                self._root.destroy()
            except Exception:
                pass
        self._root = None
        self._label = None
        self._photo = None

    def _create_window(self) -> None:
        import tkinter

        try:
            self._root = tkinter.Tk()
        except Exception as error:  # no DISPLAY or missing python3-tk
            raise DepthWindowError(str(error)) from error
        self._root.title(self._title)
        self._root.protocol('WM_DELETE_WINDOW', self.close)
        self._root.geometry(self._placement())
        if self._always_on_top:
            # The MuJoCo window is raised whenever the user interacts with it,
            # which would otherwise hide this small preview behind it.
            try:
                self._root.attributes('-topmost', True)
            except Exception:
                pass
        self._label = tkinter.Label(self._root)
        self._label.pack()
        self._root.lift()

    def _placement(self) -> str:
        """Place the window in the bottom right corner of the screen."""
        width, height = self.window_size
        screen_width = self._root.winfo_screenwidth()
        screen_height = self._root.winfo_screenheight()
        x = max(0, screen_width - width - self._margin)
        y = max(0, screen_height - height - self._margin)
        return f'{width}x{height}+{x}+{y}'

    def _to_image(self, depth: np.ndarray):
        from PIL import Image

        image = Image.fromarray(self.depth_to_pixels(depth))
        size = (self._width * self._scale, self._height * self._scale)
        return image.resize(size, Image.Resampling.NEAREST)

    def depth_to_pixels(self, depth: np.ndarray) -> np.ndarray:
        """Map metres to 8-bit grey levels, where near objects are brighter."""
        clipped = np.nan_to_num(
            depth, nan=self._max_range, posinf=self._max_range, neginf=0.0)
        clipped = np.clip(clipped, 0.0, self._max_range)
        return (255.0 * (1.0 - clipped / self._max_range)).astype(np.uint8)
