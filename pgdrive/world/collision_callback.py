def pg_collision_callback():
    """
    All collision callback should be here, and a notify() method can turn it on
    It may lower the performance if overdone
    """
    node0 = contact.getNode0().getName()
    node1 = contact.getNode1().getName()
    name = [node0, node1]
    name.remove(BodyName.Ego_vehicle)
    if name[0] in [BodyName.Traffic_vehicle, BodyName.Ego_vehicle]:
        self.crash_vehicle = True
    elif name[0] in [BodyName.Traffic_cone, BodyName.Traffic_triangle]:
        node = contact.getNode0() if contact.getNode0().hasPythonTag(name[0]) else contact.getNode1()
        self.crash_object = True if not node.getPythonTag(name[0]).crashed else False
        self._frame_objects_crashed.append(node.getPythonTag(name[0]))
    logging.debug("Crash with {}".format(name[0]))