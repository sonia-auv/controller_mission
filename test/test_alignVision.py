#!/usr/bin/env python

# unit test to test outputs of align_submarine method
# for the test : go in AlignToVision class and uncomment the return statement in
# align_submarine method and comment call of set_local_target method


PKG = 'test_align_vision'

import unittest
from controller_mission.state.alignToVision import AlignToVision
from proc_image_processing.msg import VisionTarget


class TestAlignVision(unittest.TestCase):

    def test1_min_value(self):
        test1 = AlignToVision()

        test1.is_align_with_heading_active = False
        test1.param_heading = 0
        test1.vision_position_y = 0
        test1.vision_position_z = 0
        result = test1.align_submarine()

        expect = (0, 0, 0)

        self.assertEquals(result, expect)

    def test2_max_value(self):
        test2 = AlignToVision()

        test2.is_align_with_heading_active = True
        test2.param_heading = 30
        test2.vision_position_y = -30
        test2.vision_position_z = 30
        result = test2.align_submarine()

        expect = (0.0, 30, -30)

        self.assertEquals(result, expect)

    def test3_nom_value(self):
        test3 = AlignToVision()

        test3.is_align_with_heading_active = True
        test3.param_heading = 34.5
        test3.vision_position_y = 10
        test3.vision_position_z = 25
        result = test3.align_submarine()

        expect = (0.0, 25, 34.5)

        self.assertEquals(result, expect)

    def test4_nom_value(self):
        test4 = AlignToVision()

        test4.is_align_with_heading_active = False
        test4.param_heading = 34.5
        test4.vision_position_y = -10
        test4.vision_position_z = 25
        result = test4.align_submarine()

        expect = (-10, 25, 0.0)

        self.assertEquals(result, expect)

    def test5_nom_value(self):
        test5 = AlignToVision()

        test5.is_align_with_heading_active = False
        test5.param_heading = 34.5
        test5.vision_position_y = -10
        test5.vision_position_z = 25

        test5.vision_is_reach_y = True
        test5.vision_is_reach_z = True

        result = test5.align_submarine()

        expect = (0.0, 0.0, 0.0)

        self.assertEquals(result, expect)


# unit test to test outputs of vision_cb method
class TestVisionCb(unittest.TestCase):

    def test1_min_value(self):
        test1 = AlignToVision()
        msg = VisionTarget()

        msg.header = 'unittest'
        msg.x = 0.0
        msg.y = 0.0
        msg.width = 30
        msg.height = 45
        msg.angle = 0
        msg.desc_1 = 'red'
        msg.desc_2 = 'unittest'

        test1.param_vision_target_width_in_meter = 0.23
        test1.param_bounding_box = 0.01
        test1.param_nb_pixel_to_victory = 300
        test1.param_threshold_width = 40
        test1.param_color = 'red'
        test1.vision_cb(msg)

        result = (test1.victory, test1.vision_is_reach_y, test1.vision_is_reach_z, test1.is_align_with_heading_active)
        expect = (False, True, True, True)

        self.assertEquals(result, expect)

    def test2_max_value(self):
        test1 = AlignToVision()
        msg = VisionTarget()

        msg.header = 'unittest'
        msg.x = 40.0
        msg.y = 40.0
        msg.width = 300
        msg.height = 45
        msg.angle = 0
        msg.desc_1 = 'red'
        msg.desc_2 = 'unittest'

        test1.param_vision_target_width_in_meter = 0.23
        test1.param_bounding_box = 0.01
        test1.param_nb_pixel_to_victory = 300
        test1.param_threshold_width = 40
        test1.param_color = 'red'
        test1.vision_cb(msg)

        result = (test1.victory, test1.vision_is_reach_y, test1.vision_is_reach_z, test1.is_align_with_heading_active)
        expect = (True, False, False, False)

        self.assertEquals(result, expect)

#test convertion pixel to meter
    def test3_max_value(self):
        test1 = AlignToVision()
        msg = VisionTarget()

        msg.header = 'unittest'
        msg.x = 200.0
        msg.y = 200.0
        msg.width = 30
        msg.height = 45
        msg.angle = 0
        msg.desc_1 = 'red'
        msg.desc_2 = 'unittest'

        test1.param_vision_target_width_in_meter = 0.23
        test1.param_bounding_box = 0.01
        test1.param_nb_pixel_to_victory = 300
        test1.param_threshold_width = 40
        test1.param_color = 'red'
        test1.vision_cb(msg)

        if test1.vision_position_y - 1.5333 < 0.0001:
            r_y = True
        else:
            r_y = False

        if test1.vision_position_z - 1.5333 < 0.0001:
            r_z = True
        else:
            r_z = False

        self.assertEquals(r_y, True)
        self.assertEquals(r_z, True)

#test convertion pixel to meter
    def test4_min_value(self):
        test1 = AlignToVision()
        msg = VisionTarget()

        msg.header = 'unittest'
        msg.x = 0.0
        msg.y = 0.0
        msg.width = 30
        msg.height = 45
        msg.angle = 0
        msg.desc_1 = 'red'
        msg.desc_2 = 'unittest'

        test1.param_vision_target_width_in_meter = 0.23
        test1.param_bounding_box = 0.01
        test1.param_nb_pixel_to_victory = 300
        test1.param_threshold_width = 40
        test1.param_color = 'red'
        test1.vision_cb(msg)

        if test1.vision_position_y - 0.0 < 0.0001:
            r_y = True
        else:
            r_y = False

        if test1.vision_position_z - 0.0 < 0.0001:
            r_z = True
        else:
            r_z = False

        self.assertEquals(r_y, True)
        self.assertEquals(r_z, True)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test', TestAlignVision)
    rostest.rosrun(PKG, 'test', TestVisionCb)


