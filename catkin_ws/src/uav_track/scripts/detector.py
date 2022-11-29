#!/usr/bin/env python

import rospy


def main():
    rospy.init_node('detector')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
