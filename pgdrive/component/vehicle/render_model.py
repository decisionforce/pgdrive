from pgdrive.engine.asset_loader import AssetLoader
from pgdrive.utils.space import Parameter

factor = 1

MODEL_TYPES = dict(
    ego=dict(
        length=4,
        width=1.5,
        height=1,
        model=dict(
            path='models/ferra/scene.gltf',
            scale=None,
            offset=None,
            heading=None,
        )
    ),
    s=dict(
        length=3.2,
        width=1.8,
        height=1.5,
        model=dict(
            path='models/new/beetle/scene.gltf',
            scale=(factor * .008, factor * .006, factor * .0062),
            offset=(-0.7, 0, factor * -0.16),
            heading=-90,
        )
    ),
    m=dict(
        length=3.9,
        width=2.0,
        height=1.3,
        model=dict(
            path='models/new/130/scene.gltf',
            scale=(factor * .0055, factor * .0046, factor * .0049),
            offset=(0, 0, factor * 0.33),
            heading=90,
        )
    ),
    l=dict(
        length=4.8,
        width=1.8,
        height=1.9,
        model=dict(
            path='models/new/lada/scene.gltf',
            scale=(factor * 1.1, factor * 1.1, factor * 1.1),
            offset=(1.1, -13.5, factor * -0.046),
            heading=223,
        )
    ),
    xl=dict(
        length=7.3,
        width=2.3,
        height=2.7,
        model=dict(
            path='models/new/truck/scene.gltf',
            scale=(factor * 0.031, factor * 0.025, factor * 0.025),
            offset=(0.35, 0, factor * 0),
            heading=0,
        )
    ),
)


class ModelBuffer:
    buffer = dict()
    loader = None

    @classmethod
    def get_model(cls, model_type, config):
        if cls.loader is None:
            cls.loader = AssetLoader.get_loader()
        if not hasattr(cls.loader, "loader"):
            # It is closed before!
            cls.loader.__init__()
        assert isinstance(model_type, str)
        assert model_type in MODEL_TYPES

        # TODO(pzh): Note that, there might be the possibility that the config is changed though the model type is
        #  still the one. In that case, we should not return the old loaded model.

        if model_type not in cls.buffer:

            print("{} not in buffer {}!".format(model_type, cls.buffer.keys()))

            # TODO(pzh): We should put ego into default config!
            model_details = MODEL_TYPES[model_type] or MODEL_TYPES["ego"]

            model_path = model_details["model"]["path"]
            model = cls.loader.loadModel(AssetLoader.file_path(model_path))

            # Set heading
            if model_details["model"].get("heading", None) is not None:
                # TODO(pzh): This is a workaround! We should change all +90, -pi/2 ... after refactoring coord.sys.
                model.setH(model_details["model"]["heading"] + 90)
            else:
                model.setH(config[Parameter.vehicle_vis_h])

            # Set scale
            if model_details["model"].get("scale", None) is not None:
                model.set_scale(model_details["model"]["scale"])
            else:
                model.set_scale(config[Parameter.vehicle_vis_scale])

            # Set offset (only applicable to traffic vehicle)
            if model_details["model"].get("offset", None) is not None:
                model.setPos(model_details["model"]["offset"])
            else:
                # TODO(pzh): What the hell is this? Should we add this config into traffic vehicles too?
                model.setZ(config[Parameter.vehicle_vis_z])
                model.setY(config[Parameter.vehicle_vis_y])
            cls.buffer[model_type] = model
        else:
            print("{} IN buffer {}!".format(model_type, cls.buffer.keys()))
        return cls.buffer[model_type]
