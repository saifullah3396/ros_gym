#!/usr/bin/env python3
"""
Airsim setup file provided by airsim.
"""

# Import this module to automatically setup path to local airsim module
# This module first tries to see if airsim module is installed via pip
# If it does then we don't do anything else
# Else we look up grand-parent folder to see if it has airsim folder
#    and if it does then we add that in sys.path

import os
import sys
import logging


class SetupPath:
    """
    This class simply tries to see if airsim exists
    """
    # pylint: disable=missing-function-docstring
    @staticmethod
    def get_dir_levels(path):
        path_norm = os.path.normpath(path)
        return len(path_norm.split(os.sep))

    @staticmethod
    def get_current_path():
        cur_filepath = __file__
        return os.path.dirname(cur_filepath)

    @staticmethod
    def get_grand_parent_dir():
        cur_path = SetupPath.get_current_path()
        if SetupPath.get_dir_levels(cur_path) >= 2:
            return os.path.dirname(os.path.dirname(cur_path))
        return ''

    @staticmethod
    def get_parent_dir():
        cur_path = SetupPath.get_current_path()
        if SetupPath.get_dir_levels(cur_path) >= 1:
            return os.path.dirname(cur_path)
        return ''

    @staticmethod
    def add_airsim_module_path():
        parent = SetupPath.get_parent_dir()
        if parent is not '':
            airsim_path = os.path.join(parent, 'airsim')
            client_path = os.path.join(airsim_path, 'client.py')
            if os.path.exists(client_path):
                sys.path.insert(0, parent)
        else:
            logging.warning(
                """airsim module not found in parent folder.
                Using installed package (pip install airsim).""")
