from pgdrive.utils.pg_config import PGConfig


def test_recursive_config():
    c = PGConfig({
        "aa": {
            "bb": {
                "cc": 100
            }
        }
    })
    assert c.aa.bb.cc == 100
    assert isinstance(c.aa, PGConfig)
    assert isinstance(c.aa.bb, PGConfig)
    assert isinstance(c.aa.bb.cc, int)

    c.TMP_update({
        "aa": {
            "bb": {
                "cc": 101
            }
        }
    })
    assert c.aa.bb.cc == 101

    try:
        c.TMP_update({
            "aa": {
                "bb": 102
            }
        })
    except TypeError:
        pass
    else:
        raise ValueError()


if __name__ == '__main__':
    test_recursive_config()
