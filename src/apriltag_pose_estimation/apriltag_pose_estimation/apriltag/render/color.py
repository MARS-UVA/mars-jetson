"""Definitions of colors."""

from dataclasses import dataclass


__all__ = ['Color', 'RED', 'YELLOW', 'GREEN', 'CYAN', 'BLUE', 'MAGENTA', 'BLACK', 'WHITE']


@dataclass(frozen=True)
class Color:
    """An RGB color."""
    red: int
    """The red component of the color."""
    green: int
    """The green component of the color."""
    blue: int
    """The blue component of the color."""

    def __post_init__(self):
        if not (0 <= self.red <= 255):
            raise ValueError('red must be between 0 and 255')
        if not (0 <= self.green <= 255):
            raise ValueError('green must be between 0 and 255')
        if not (0 <= self.blue <= 255):
            raise ValueError('blue must be between 0 and 255')

    def bgr(self) -> tuple[int, int, int]:
        """Returns a BGR tuple for the color."""
        return self.blue, self.green, self.red

    def rgb(self) -> tuple[int, int, int]:
        """Returns an RGB tuple for the color."""
        return self.red, self.green, self.blue


RED = Color(255, 0, 0)
YELLOW = Color(255, 255, 0)
GREEN = Color(0, 255, 0)
CYAN = Color(0, 255, 255)
BLUE = Color(0, 0, 255)
MAGENTA = Color(255, 0, 255)
BLACK = Color(0, 0, 0)
WHITE = Color(255, 255, 255)
