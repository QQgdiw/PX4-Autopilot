#!/usr/bin/env python3
"""Static message/translation contract checks; not a ROS/DDS interoperability test."""

import pathlib
import re
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
NEW = ROOT / "msg/versioned/VehicleLocalPosition.msg"
OLD = ROOT / "msg/px4_msgs_old/msg/VehicleLocalPositionV0.msg"
TRANSLATION = ROOT / "msg/translation_node/translations/translation_vehicle_local_position_v1.h"


def fields(path):
    return [line.split("#", 1)[0].strip() for line in path.read_text().splitlines()
            if line.split("#", 1)[0].strip() and "=" not in line.split("#", 1)[0]]


class LocalPositionContract(unittest.TestCase):
    def test_version_and_layout(self):
        self.assertIn("uint32 MESSAGE_VERSION = 0", OLD.read_text())
        self.assertIn("uint32 MESSAGE_VERSION = 1", NEW.read_text())
        new_fields = fields(NEW)
        new_fields.remove("bool heading_good_for_local_control")
        self.assertEqual(new_fields, fields(OLD))

    def test_complete_bidirectional_copy(self):
        text = TRANSLATION.read_text()
        names = [field.split()[1] for field in fields(OLD)]
        for name in names:
            self.assertEqual(text.count(f"msg_newer.{name} = msg_older.{name};"), 1)
            self.assertEqual(text.count(f"msg_older.{name} = msg_newer.{name};"), 1)
        self.assertEqual(len(re.findall(r"msg_newer\.\w+ =", text)), len(names) + 1)
        self.assertEqual(len(re.findall(r"msg_older\.\w+ =", text)), len(names))
        self.assertIn("msg_newer.heading_good_for_local_control = false;", text)

    def test_versioned_dds_topic(self):
        # The yaml must retain the base name: firmware appends MESSAGE_VERSION.
        yaml = (ROOT / "src/modules/uxrce_dds_client/dds_topics.yaml").read_text()
        self.assertIn("topic: /fmu/out/vehicle_local_position\n", yaml)
        self.assertNotIn("topic: /fmu/out/vehicle_local_position_v1\n", yaml)
        header = (ROOT / "src/modules/uxrce_dds_client/dds_topics.h.em").read_text()
        self.assertIn("get_message_version<", header)
        utility = (ROOT / "src/modules/uxrce_dds_client/utilities.hpp").read_text()
        self.assertIn('"_v%u"', utility)


if __name__ == "__main__":
    unittest.main()
