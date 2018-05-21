#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date: 21/02/2018
# @info: Picker - a simple picker class
# ----------------------------------

import rospy
import geometry_msgs.msg
import tf
import math


class Picker(object):
    """Picker class definition"""
    def __init__(self, name, env, farm, tray_capacity, max_n_trays,
                 picking_rate, transport_rate, loading_time):
        """Create a Picker object

        Keyword arguments:
        name -- name/id of the picker
        env -- simpy.Environment
        farm -- provides the allocation and monitor process as well as a graph.
        """
        self.name = name
        self.env = env
        self.farm = farm
        self.n_trays = 0     # current number of trays with the picker
        self.tot_trays = 0   # total number of trays by the picker
        self.tray_capacity = tray_capacity
        self.picking_rate = picking_rate
        self.transport_rate = transport_rate
        self.max_n_trays = max_n_trays
        self.max_wait_for_allocation = 5    # max time to wait before unloading everything

        self.loading_time = loading_time    # time spent at localStorage
        self.assigned_row = None            # only for picking mode

        self.mode = 0       # 0:free, 1:picking, 2:transporting, 3:finished_job
        self.curr_node = None
        self.local_storage_node = None
        self.picking_dir = None     # "forward" or "reverse"

        self.curr_row = None
        self.prev_row = None

        # [head_node, start_node, end_node, row_node_dist, last_node_dist, local_storage_node]
        self.curr_row_info = []
        self.prev_row_info = []

        self.row_path = []

        self.picking_progress = 0.  # percentage of tray_capacity

        self.transport_progress = 0.
        self.pose_pub = rospy.Publisher('/%s/pose' %(self.name), geometry_msgs.msg.Pose, queue_size=10)
        self.pose = geometry_msgs.msg.Pose()

        self.action = self.env.process(self.picking_process())

    def picking_process(self, ):
        """ Picker's picking process
        """
        # 1. picker should report for duty first
        self.farm.picker_report(self.name)
        while True:
            # 2. If the picker is assigned a row,
            #   a. continue picking
            #   b. yield along each row_node_dist
            #   c. check tray_cap,
            #       i. if tray_cap is reached, increment n_trays
            #       ii. if n_trays reach max_n_trays, start the transport_process
            #       ii. if not move to the next row_node in the next iter
            if self.mode == 1:
                # move along each node in the assigned row
                # the picker is at the curr_node
                # move to the node next to curr_node in the row_path
                if self.picking_dir is "forward":
                    curr_node_index = self.row_path.index(self.curr_node)
                    next_node = self.row_path[curr_node_index + 1]
                    if curr_node_index != len(self.row_path) - 2:
                        node_dist = self.curr_row_info[3]
                    else:
                        node_dist = self.curr_row_info[4]

                    time_to_pick = node_dist / self.picking_rate
                    yield self.env.timeout(time_to_pick)

                    # update the picking progress
                    self.picking_progress += self.farm.graph.nodes[self.curr_node].yield_at_node
                    print("%s reached %s from %s at %0.3f" %(self.name, next_node,
                                                              self.curr_node,
                                                              self.env.now))
                    print("%s : tot_trays: %02d, n_trays: %02d, pick_progress: %0.3f" %(self.name,
                                                                                        self.tot_trays,
                                                                                        self.n_trays,
                                                                                        self.picking_progress))
                    self.curr_node = "" + next_node
                    # publish pose
                    self.publish_pose(self.curr_node, 0.)

                    if self.curr_node == self.row_path[-1]:
                        self.picking_dir = "reverse"
                        print("%s changing to reverse along %s at %0.3f" %(self.name,
                                                                            self.curr_row,
                                                                            self.env.now))

                    # if the tray capacity is reached, increment n_trays
                    if self.picking_progress >= self.tray_capacity:
                        self.n_trays += 1
                        self.picking_progress -= self.tray_capacity

                    # if max_n_trays is reached
                    if self.n_trays == self.max_n_trays:
                        # transport to local storage and return
                        trans_path, trans_path_dist = self.farm.graph.get_path_details(self.curr_node,
                                                                                       self.local_storage_node)
                        time_to_transport = trans_path_dist / self.transport_rate
                        self.mode = 2
                        print("%s reached %d trays. going to local storage at %0.3f" %(self.name,
                                                                                        self.max_n_trays,
                                                                                        self.env.now))
                        print("%s : tot_trays: %02d, n_trays: %02d, pick_progress: %0.3f" %(self.name,
                                                                                            self.tot_trays,
                                                                                            self.n_trays,
                                                                                            self.picking_progress))
                        yield self.env.process(self.transport_process(time_to_transport, 2))
                        print("%s returned from local storage at %0.3f" %(self.name,
                                                                           self.env.now))

                        # resume picking
                        self.tot_trays += self.max_n_trays
                        self.n_trays -= self.max_n_trays
                        self.mode = 1

                elif self.picking_dir is "reverse":
                    # work with negative indices
                    curr_node_index = self.row_path.index(self.curr_node) - len(self.row_path)
                    next_node = self.row_path[curr_node_index - 1]
                    if curr_node_index != -1:
                        node_dist = self.curr_row_info[3]
                    else:
                        node_dist = self.curr_row_info[4]

                    time_to_pick = node_dist / self.picking_rate
                    yield self.env.timeout(time_to_pick)

                    # update the picking progress
                    self.picking_progress += self.farm.graph.nodes[self.curr_node].yield_at_node
                    print("%s reached %s from %s at %0.3f" %(self.name, next_node,
                                                              self.curr_node,
                                                              self.env.now))
                    print("%s : tot_trays: %02d, n_trays: %02d, pick_progress: %0.3f" %(self.name,
                                                                                        self.tot_trays,
                                                                                        self.n_trays,
                                                                                        self.picking_progress))

                    self.curr_node = "" + next_node
                    # publish pose
                    self.publish_pose(self.curr_node, math.pi/2)

                    if self.curr_node == self.row_path[0]:
                        # row is finished
                        self.farm.finished_rows[self.curr_row].succeed(value=self.env.now)

                        self.picking_dir = None
                        self.prev_row = "" + self.curr_row
                        self.prev_row_info = [] + self.curr_row_info
                        self.curr_row = None
                        self.row_path = []

                    # check picking progress
                    if self.picking_progress >= self.tray_capacity:
                        self.picking_progress -= self.tray_capacity
                        self.n_trays += 1

                        if self.n_trays == self.max_n_trays:
                            if self.curr_row is None:
                                # transport to local storage. no need to return as row is finished
                                trans_path, trans_path_dist = self.farm.graph.get_path_details(self.curr_node,
                                                                                               self.local_storage_node)
                                time_to_transport = trans_path_dist / self.transport_rate
                                self.mode = 2
                                print("%s reached %d trays. going to local storage at %0.3f" %(self.name,
                                                                                                self.max_n_trays,
                                                                                                self.env.now))
                                yield self.env.process(self.transport_process(time_to_transport, 1))
                                # finished the allocated row and transported all berries
                                # now at local_storage_node
                                # no current allocation - change mode to zero
                                self.tot_trays += self.max_n_trays
                                self.n_trays -= self.max_n_trays
                                self.curr_node = "" + self.local_storage_node
                                self.mode = 0
                            else:
                                # transport to local storage and return
                                trans_path, trans_path_dist = self.farm.graph.get_path_details(self.curr_node,
                                                                                               self.local_storage_node)
                                time_to_transport = trans_path_dist / self.transport_rate
                                self.mode = 2
                                print("%s reached %d trays. going to local storage at %0.3f" %(self.name,
                                                                                                self.max_n_trays,
                                                                                                self.env.now))
                                yield self.env.process(self.transport_process(time_to_transport, 2))
                                print("%s returned from local storage at %0.3f" %(self.name,
                                                                                   self.env.now))

                                # resume picking
                                self.tot_trays += self.max_n_trays
                                self.n_trays -= self.max_n_trays
                                self.mode = 1
                    else:
                        # picking_progress != tray_capacity : go to not allocated mode
                        if self.curr_row is None:
                            # no current allcoations
                            self.mode = 0

            # 3. If in mode free, check if there is any new assignments
            #       If there is no new assignment and no rows left to be assigned, finish picking
            #       If there is a new assignment
            #           a. Move to the start node of the path
            #           b. Get the path from the current loc to start_node of new row
            #           c. Move at transport_rate to start_node
            #           d. Continue/Start picking along the new row by changing mode to picking
            elif self.mode == 0:
                row_id = self.farm.curr_picker_allocations[self.name]
                row_id = None if row_id == self.prev_row else row_id

                if (row_id is None) and (len(self.farm.unallocated_rows) == 0):
                    if self.curr_node == self.local_storage_node:
                        # at local storage after unloading max_n_trays
                        # if there is anything left, unload those and leave the picking process
                        if (self.n_trays > 0) or (self.picking_progress > 0.):
                            self.env.process(self.transport_process(0, 1))
                            self.tot_trays += self.n_trays + self.picking_progress / self.tray_capacity
                            self.n_trays = 0
                            self.picking_progress = 0
                        self.mode = 3
                    elif self.curr_node is not None:
                        # not the first allocation, meaning the picker is at some node already
                        if (self.n_trays > 0) or (self.picking_progress > 0.):
                            trans_path, trans_path_dist = self.farm.graph.get_path_details(self.curr_node,
                                                                                           self.local_storage_node)
                            time_to_transport = trans_path_dist / self.transport_rate
                            self.mode = 2
                            self.env.process(self.transport_process(time_to_transport, 1))
                            self.tot_trays += self.n_trays + self.picking_progress / self.tray_capacity
                            self.n_trays = 0
                            self.picking_progress = 0

                        self.mode = 3
                    # finish the picking process
                    print("%s finishing picking process. all rows are assigned" %(self.name))
                    break

                elif row_id is not None: # if there is a row assigned to the picker
                    self.curr_row = row_id
                    self.curr_row_info = self.farm.row_info[self.curr_row]

                    # set local storage as curr_node if never assigned before
                    if self.curr_node is None:
                        self.curr_node = self.curr_row_info[5]
                        self.local_storage_node = self.curr_row_info[5]
                        
                        # publish pose
                        self.publish_pose(self.curr_node, 0.)

                    print("%s is moving to the start of %s at %0.3f" %(self.name,
                                                                        self.curr_row,
                                                                        self.env.now))
                    # transport to the start_node of the row
                    trans_path, trans_path_dist = self.farm.graph.get_path_details(self.curr_node,
                                                                                   self.curr_row_info[1])
                    time_to_transport = trans_path_dist / self.transport_rate
                    yield self.env.timeout(time_to_transport)

                    # picker moved to the start_node of the row (yield above)
                    self.curr_node = self.curr_row_info[1]
                    self.row_path = self.farm.graph.get_path(self.curr_node, self.curr_row_info[2])

                    # publish pose
                    self.publish_pose(self.curr_node, 0.)

                    print("%s started forward picking on %s at %0.3f" %(self.name, row_id, self.env.now))
                    # change current mode to picking
                    self.mode = 1 # picking mode

                    self.picking_dir = "forward"

            yield self.env.timeout(0.001)

    def transport_process(self, time_to_transport, n_times):
        """Picker's transportation process
        """
        # This node should ideally implement picker's transportation to the local storage
        # and return if needed.
        # Only a timeout implementation is done now.
        #   1. move along the path (yield timeout(time_to_travel_path))
        yield self.env.timeout(time_to_transport)
        # publish pose
        self.publish_pose(self.local_storage_node, 0.0)
        #   2. request for the local storage access
        #   3. wait further for unloading (yield timeout(loading_time))
        print("%s requesting for local_storage resource at %0.3f" %(self.name, self.env.now))
        with self.farm.local_storages[self.local_storage_node].request() as req:
            yield req
            print("%s got access to local_storage resource at %0.3f" %(self.name, self.env.now))
            yield self.env.timeout(self.loading_time)

            print("%s spent %0.3f for unloading at the local_staorage" %(self.name, self.loading_time))
        # if needed, return to previous node
        if n_times == 2:
            yield self.env.timeout(time_to_transport)
            # publish pose
            if self.picking_dir is "forward":
                self.publish_pose(self.curr_node, 0.)
            else:
                self.publish_pose(self.curr_node, math.pi/2)

    def publish_pose(self, node, z_orientation):
        """This method publishes the current position of the picker. Called only at nodes"""
        self.pose.position.x = self.farm.graph.nodes[node].x
        self.pose.position.y = self.farm.graph.nodes[node].y
        self.pose.position.z = 0.0
        quaternion = tf.transformations.quaternion_from_euler(0., 0., z_orientation)
        self.pose.orientation.x = quaternion[0]
        self.pose.orientation.y = quaternion[1]
        self.pose.orientation.z = quaternion[2]
        self.pose.orientation.w = quaternion[3]
        self.pose_pub.publish(self.pose)
