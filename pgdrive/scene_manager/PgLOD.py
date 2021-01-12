from pgdrive.world.pg_world import PgWorld


class PgLOD:
    """
    Used to cull distant rendering object to improve rendering efficiency
    TODO calculation efficiency can also be improved in the future
    """
    LOD_DIST = 100

    @classmethod
    def cull_distant_blocks(cls, blocks: list, pos, pg_world: PgWorld):
        # A distance based LOD rendering like GTA
        for block in blocks:
            if block.bounding_box[0] - cls.LOD_DIST < pos[0] < block.bounding_box[1] + cls.LOD_DIST and \
                    block.bounding_box[2] - cls.LOD_DIST < \
                    pos[1] < block.bounding_box[3] + cls.LOD_DIST:
                if not block.node_path.hasParent():
                    block.node_path.reparentTo(pg_world.worldNP)
                    for node in block.dynamic_nodes:
                        pg_world.physics_world.dynamic_world.attach(node)
            else:
                if block.node_path.hasParent():
                    block.node_path.detachNode()
                    for node in block.dynamic_nodes:
                        pg_world.physics_world.dynamic_world.remove(node)
