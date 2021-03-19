from pgdrive.constants import BodyName
import logging

def pg_collision_callback(contact):
    """
    All collision callback should be here, and a notify() method can turn it on
    It may lower the performance if overdone
    """

    # now it only process BaseVehicle collision
    node0 = contact.getNode0()
    node1 = contact.getNode1()
    if not (node0.hasPythonTag(BodyName.Ego_vehicle) or node1.hasPythonTag(BodyName.Ego_vehicle)):
        return

    nodes = [node0, node1]
    another_nodes = [node1, node0]
    for i in range(2):
        if nodes[i].hasPythonTag(BodyName.Ego_vehicle):
            if another_nodes[i].getName() in [BodyName.Traffic_vehicle, BodyName.Ego_vehicle]:
                nodes[i].getPythonTag(BodyName.Ego_vehicle).crash_vehicle = True
            elif another_nodes[i].getName() in [BodyName.Traffic_cone, BodyName.Traffic_triangle]:
                nodes[i].getPythonTag(BodyName.Ego_vehicle).crash_object = True
            elif another_nodes[i].getName() == BodyName.Sidewalk:
                nodes[i].getPythonTag(BodyName.Ego_vehicle).crash_sidewalk = True
            # TODO update this
            # self._frame_objects_crashed.append(node.getPythonTag(name[0]))
            logging.debug("{} crash with {}".format(nodes[i].getName(),another_nodes[i].getName()))