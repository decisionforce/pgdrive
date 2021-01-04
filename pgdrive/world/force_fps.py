import time


class ForceFPS:
    UNLIMITED = 0
    # FPS60 = 1
    FORCED = 2

    def __init__(self, pg_world, start=False):
        fps = 1 / pg_world.pg_config["physics_world_step_size"]
        self.pg_world = pg_world
        self.last = time.time()
        self.init_fps = fps
        if start:
            self.state = self.FORCED
            self.fps = fps
        else:
            self.state = self.UNLIMITED
            self.fps = None

    @property
    def interval(self):
        return 1 / self.fps if self.fps is not None else None

    def __enter__(self):
        # print("Force fps, now: ", self.last)
        now = time.time()
        if self.interval and now - self.last < self.interval:
            time.sleep(self.interval - (now - self.last))

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.last = time.time()

    def toggle(self):
        if self.state == self.UNLIMITED:
            self.pg_world.taskMgr.add(self.force_fps_task, "force_fps")
            self.state = self.FORCED
            self.fps = self.init_fps
        elif self.state == self.FORCED:
            self.pg_world.taskMgr.remove("force_fps")
            self.state = self.UNLIMITED
            self.fps = None

    def force_fps_task(self, task):
        with self:
            pass
        return task.cont

    def __call__(self, *args, **kwargs):
        return self.state == self.FORCED
