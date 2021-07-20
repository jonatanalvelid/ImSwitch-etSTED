import glob
import os
from abc import ABC
from pathlib import Path
from shutil import copy2


def getSystemUserDir():
    """ Returns the user's documents folder if they are using a Windows system,
    or their home folder if they are using another operating system. """

    if os.name == 'nt':  # Windows system, try to return documents directory
        try:
            import ctypes.wintypes
            CSIDL_PERSONAL = 5  # Documents
            SHGFP_TYPE_CURRENT = 0  # Current value

            buf = ctypes.create_unicode_buffer(ctypes.wintypes.MAX_PATH)
            ctypes.windll.shell32.SHGetFolderPathW(0, CSIDL_PERSONAL, 0, SHGFP_TYPE_CURRENT, buf)

            return buf.value
        except ImportError:
            pass

    return os.path.expanduser('~')  # Non-Windows system, return home directory

_baseUserFilesDir = os.path.join(getSystemUserDir(), 'ImSwitch')


class FileDirs(ABC):
    """ Base class for directory catalog classes. """

    @classmethod
    def list(cls):
        """ Returns all directories in the catalog. """
        return [cls.__dict__.get(name) for name in dir(cls) if (
            not callable(getattr(cls, name)) and not name.startswith('_')
        )]


class UserFileDirs(FileDirs):
    """ Catalog of directories that contain user configuration files. """
    Root = _baseUserFilesDir
    Config = os.path.join(_baseUserFilesDir, 'config')


# Copyright (C) 2020, 2021 TestaLab
# This file is part of ImSwitch.
#
# ImSwitch is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# ImSwitch is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
