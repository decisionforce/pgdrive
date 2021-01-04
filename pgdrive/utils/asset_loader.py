import logging
import pathlib
import os
import sys




class AssetLoader:
    """
    Load model for each element when render is needed.
    """
    loader = None
    asset_path = None

    @staticmethod
    def init_loader(pg_world):
        """
        Due to the feature of Panda3d, keep reference of loader in static variable
        """
        root_path = pathlib.PurePosixPath(__file__).parent.parent
        # path = AssetLoader.windows_style2unix_style(root_path) if sys.platform == "win32" else root_path
        AssetLoader.asset_path = root_path.joinpath("assets")
        if pg_world.win is None:
            logging.debug("Physics world mode")
            return
        logging.debug("Onscreen/Offscreen mode, Render/Load Elements")
        AssetLoader.loader = pg_world.loader

    @classmethod
    def get_loader(cls):
        assert AssetLoader.loader, "Initialize AssetLoader before getting it"
        return cls.loader

    @staticmethod
    def windows_style2unix_style(path):
        u_path = "/" + path[0].lower() + path[2:]
        u_path.replace("\\", "/")
        return u_path

    @staticmethod
    def file_path(*path_string):
        path=AssetLoader.asset_path.join
        if sys.platform.startswith("win"):
            path = path.replace("\\", "/")
        return path
