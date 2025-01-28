"""StrEnum helper class.

This code is necessary because the `StrEnum` class was only added in python 3.11, but ROS Humble uses Python 3.10.

Source obtained from https://github.com/irgeek/StrEnum/tree/master, which is licensed under the MIT License.

Copyright (c) 2019 James C Sinclair

Changes made to original contents (https://github.com/irgeek/StrEnum/blob/master/strenum/__init__.py):
- Only included StrEnum class and necessary imports.
- Made the StrEnum member names lower case to match behaviour of Python 3.11 implementation.
- Updated documentation string.
"""

# ruff: noqa

import enum


class StrEnum(str, enum.Enum):
    """
    StrEnum is a Python ``enum.Enum`` that inherits from ``str``. The default
    ``auto()`` behavior uses the member name as its value.

    Example usage::

        class Example(StrEnum):
            UPPER_CASE = auto()
            lower_case = auto()
            MixedCase = auto()

        assert Example.UPPER_CASE == "upper_case"
        assert Example.lower_case == "lower_case"
        assert Example.MixedCase == "mixedcase"
    """

    def __new__(cls, value, *args, **kwargs):
        if not isinstance(value, (str, enum.auto)):
            raise TypeError(f"Values of StrEnums must be strings: {value!r} is a {type(value)}")
        return super().__new__(cls, value, *args, **kwargs)

    def __str__(self):
        return str(self.value)

    def _generate_next_value_(name, *_):
        return name.lower()
