import logging

from pgdrive.constants import BodyName


def pg_collision_callback(contact):
    """
    All collision callback should be here, and a notify() method can turn it on
    It may lower the performance if overdone
    """

    # now it only process BaseVehicle collision
    node0 = contact.getNode0()
    node1 = contact.getNode1()

    nodes = [node0, node1]
    another_nodes = [node1, node0]
    for i in range(2):
        if nodes[i].hasPythonTag(BodyName.Ego_vehicle):
            another_node_name = another_nodes[i].getName()
            # crash vehicles
            if another_node_name in [BodyName.Traffic_vehicle, BodyName.Ego_vehicle]:
                nodes[i].getPythonTag(BodyName.Ego_vehicle).crash_vehicle = True
            # crash objects
            elif another_node_name == BodyName.Traffic_cone and not another_nodes[i].getPythonTag(BodyName.Traffic_cone
                                                                                                  ).crashed:
                nodes[i].getPythonTag(BodyName.Ego_vehicle).crash_object = True
                if another_nodes[i].getPythonTag(BodyName.Traffic_cone).COST_ONCE:
                    another_nodes[i].getPythonTag(BodyName.Traffic_cone).crashed = True
            elif another_node_name == BodyName.Traffic_triangle and not another_nodes[i].getPythonTag(
                    BodyName.Traffic_triangle).crashed:
                nodes[i].getPythonTag(BodyName.Ego_vehicle).crash_object = True
                if another_nodes[i].getPythonTag(BodyName.Traffic_cone).COST_ONCE:
                    another_nodes[i].getPythonTag(BodyName.Traffic_triangle).crashed = True
            logging.debug("{} crash with {}".format(nodes[i].getName(), another_nodes[i].getName()))
