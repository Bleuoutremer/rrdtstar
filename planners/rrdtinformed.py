import logging
import random

from overrides import overrides
import pygame
from checkCollision import *
from helpers import *
from planners.particleFilterSampler import (ENERGY_START,
                                            RANDOM_RESTART_PARTICLES_ENERGY_UNDER,
                                            Particle, ParticleFilterSampler)
from planners.rrtPlanner import RRTPlanner
from planners.randomPolicySampler import RandomPolicySampler

LOGGER = logging.getLogger(__name__)

MAX_NUMBER_NODES = 20000


def update_progress(progress, total_num, num_of_blocks=10):
    if not logging.getLogger().isEnabledFor(logging.INFO):
        return
    percentage = progress / total_num
    print(
        '\r[{bar:<{num_of_blocks}}] {cur}/{total} {percen:0.1f}%'.format(
            bar='#' * int(percentage * num_of_blocks),
            cur=progress,
            total=total_num,
            percen=percentage * 100,
            num_of_blocks=num_of_blocks),
        end='')
    if percentage == 1:
        print()


class BFS:
    """Walk through the connected nodes with BFS"""

    def __init__(self, node, validNodes):
        self.visitedNodes = set()
        self.validNodes = validNodes
        self.next_node_to_visit = [node]
        self.next_node = None

    def visit_node(self, node):
        self.visitedNodes.add(node)
        self.next_node_to_visit.extend(node.edges)
        self.next_node = node

    def has_next(self):
        if self.next_node is not None:
            return True
        if len(self.next_node_to_visit) < 1:
            return False
        # get next available node
        while True:
            _node = self.next_node_to_visit.pop(0)
            if _node not in self.visitedNodes and _node in self.validNodes:
                break
            if len(self.next_node_to_visit) < 1:
                return False
        self.visit_node(_node)
        return True

    def next(self):
        node = self.next_node
        self.next_node = None
        return node


class TreesManager:
    def __init__(self, args, restart_when_merge):
        self.root = None
        self.disjointedTrees = []
        self.args = args
        self.restart_when_merge = restart_when_merge

    def connect_two_nodes(self, newnode, nn, parent_tree=None,
                          draw_only=False):
        """Add node to disjoint tree OR root tree. Draw line for it too."""
        if not draw_only:
            if parent_tree is self.root:
                # using rrt* algorithm to add each nodes
                newnode, nn = self.args.planner.rrt_star_add_node(newnode, nn)
            else:
                newnode.edges.append(nn)
                nn.edges.append(newnode)
            if parent_tree is not None:
                parent_tree.add_newnode(newnode)
        self.args.env.draw_path(newnode, nn)
        return newnode, nn

    def add_pos_to_existing_tree(self, newnode, parent_tree):
        """Try to add pos to existing tree. If success, return True."""
        nearest_nodes = self.find_nearest_node_from_neighbour(
            node=newnode, parent_tree=parent_tree, radius=self.args.radius)
        for nearest_neighbour_node, nearest_neighbour_tree in nearest_nodes:
            if self.args.env.cc.path_is_free(newnode.pos,
                                        nearest_neighbour_node.pos):
                if parent_tree is None:
                    ### joining ORPHAN NODE to a tree
                    self.connect_two_nodes(newnode, nearest_neighbour_node,
                                           nearest_neighbour_tree)
                    parent_tree = nearest_neighbour_tree
                    LOGGER.debug(
                        " ==> During respawning particle, joining to existing tree with size: {}"
                        .format(len(nearest_neighbour_tree.nodes)))
                else:
                    ### joining a TREE to another tree
                    try:
                        parent_tree = self.join_trees(
                            parent_tree,
                            nearest_neighbour_tree,
                            tree1_node=newnode,
                            tree2_node=nearest_neighbour_node)
                    except AssertionError as e:
                        LOGGER.warning(
                            "== Assertion error in joining sampled point to existing tree... Skipping this node..."
                        )
        return parent_tree

    def find_nearest_node_from_neighbour(self, node, parent_tree, radius):
        """
        Given a tree, a node within that tree, and radius
        Return a list of cloest nodes (and its corresponding tree) within the radius (that's from other neighbourhood trees)
        Return None if none exists
        IF root exists in the list, add it at the last position (So the connection behaviour would remain stable)
            This ensure all previous action would only add add edges to each nodes, and only the last action would it
            modifies the entire tree structures wtih rrt* procedures.
        """
        nearest_nodes = {}
        for tree in [*self.disjointedTrees, self.root]:
            if tree is parent_tree:
                # skip self
                continue
            idx = self.args.planner.find_nearest_neighbour_idx(
                node.pos, tree.poses[:len(tree.nodes)])
            nn = tree.nodes[idx]
            if dist(nn.pos, node.pos) < radius:
                nearest_nodes[tree] = nn
        # construct list of the found solution. And root at last (or else the result won't be stable)
        root_nn = nearest_nodes.pop(self.root, None)
        nearest_nodes_list = [(nearest_nodes[key], key)
                              for key in nearest_nodes]
        if root_nn is not None:
            nearest_nodes_list.append((root_nn, self.root))
        return nearest_nodes_list

    def join_tree_to_root(self, tree, middle_node):
        """It will join the given tree to the root"""
        from env import Colour
        bfs = BFS(middle_node, validNodes=tree.nodes)
        # add all nodes from disjoint tree via rrt star method
        total_num = len(tree.nodes)
        progress = 0
        LOGGER.info("> Joining to root tree")
        while bfs.has_next():
            newnode = bfs.next()
            progress += 1
            update_progress(progress, total_num, num_of_blocks=20)
            # draw white (remove edge for visual) on top of disjointed tree
            for e in (x for x in newnode.edges
                      if x not in bfs.visitedNodes and x in bfs.validNodes):
                self.args.env.draw_path(e, newnode, Colour.white)
            try:
                self.connect_two_nodes(newnode, nn=None, parent_tree=self.root)
            except LookupError:
                LOGGER.warning(
                    "nn not found when attempting to joint to root. Ignoring..."
                )
            # remove this node's edges (as we don't have a use on them anymore) to free memory
            del newnode.edges

        assert progress == total_num, "Inconsistency in BFS walk {} != {}".format(
            progress, total_num)

        # raise Exception("NOT implemented yet")

    def join_trees(self, tree1, tree2, tree1_node, tree2_node):
        """
        Join the two given tree together (along with their nodes).
        It will delete the particle reference from the second tree.
        It will use RRT* method to add all nodes if one of the tree is the ROOT.

        tree1_node & 2 represent the nodes that join the two tree together. It only matters currently to
        joining root tree to disjointed treeself.

        Return the tree that has not been killed
        """
        assert tree1 is not tree2, "Both given tree should not be the same"
        if tree1 not in self.disjointedTrees:
            assert tree1 is self.root, "Given tree is neither in disjointed tree, nor is it the root: {}".format(
                tree1)
        if tree2 not in self.disjointedTrees:
            assert tree2 is self.root, "Given tree is neither in disjointed tree, nor is it the root: {}".format(
                tree2)

        LOGGER.info(" => Joining trees with size {} to {}".format(
            len(tree1.nodes), len(tree2.nodes)))
        # Re-arrange only. Make it so that tree1 will always be root (if root exists among the two)
        # And tree1 node must always be belong to tree1, tree2 node belong to tree2
        if tree1 is not self.root:
            # set tree1 as root (if root exists among the two)
            tree1, tree2 = tree2, tree1
        if tree1_node in tree2.nodes or tree2_node in tree1.nodes:
            # swap to correct position
            tree1_node, tree2_node = tree2_node, tree1_node
        assert tree1_node in tree1.nodes, "Given nodes does not belong to the two given corresponding trees"
        assert tree2_node in tree2.nodes, "Given nodes does not belong to the two given corresponding trees"

        if tree1 is self.root:
            # find which middle_node belongs to the disjointed tree
            self.join_tree_to_root(tree2, tree2_node)
            self.connect_two_nodes(tree1_node, tree2_node, draw_only=True)
        else:
            self.connect_two_nodes(tree1_node, tree2_node)
            tree1.extend_tree(tree2)
        del tree2.nodes
        del tree2.poses
        self.disjointedTrees.remove(tree2)

        if self.restart_when_merge:
            # restart all particles
            for p in tree2.particle_handler:
                p.restart()
            del tree2.particle_handler
        else:
            # pass the remaining particle to the remaining tree
            for p in tree2.particle_handler:
                p.tree = tree1
                tree1.particle_handler.append(p)
        return tree1



RANDOM_RESTART_EVERY = 20
ENERGY_START = 10
RANDOM_RESTART_PARTICLES_ENERGY_UNDER = 0.75


class DisjointTreeParticle(Particle):
    @overrides
    def __init__(self,
                 tree_manager,
                 p_manager,
                 direction=None,
                 pos=None,
                 isroot=False,
                 startPtNode=None):
        self.isroot = isroot
        self.p_manager = p_manager
        self.tree_manager = tree_manager
        self.last_node = None
        if isroot:
            self.tree_manager.root = TreeRoot(particle_handler=self)
            self.tree = self.tree_manager.root
            self.tree.add_newnode(startPtNode)
        super().__init__(direction=direction, pos=pos)

    @overrides
    def restart(self, direction=None, pos=None, restart_when_merge=True, specific_pos=None):
        if self.isroot:
            # root particles has a different initialisation method
            # (for the first time)
            self.isroot = False
            super().restart(direction, pos)
            return
        self.last_node = None
        merged_tree = None
        if pos is None:
            # get random position
            pos = self.p_manager.new_pos_in_free_space(specific_pos)
            merged_tree = self.tree_manager.add_pos_to_existing_tree(
                Node(pos), None)
            if merged_tree is not None and restart_when_merge:
                # Successfully found a new valid node that's close to existing tree
                # Return False to indicate it (and abort restart if we want more exploration)
                self.p_manager.add_to_restart(self)
                # we need to abort the restart procedure. add this to pending restart
                return False
        try:
            self.tree.particle_handler.remove(self)
        except AttributeError:
            # probably this is its first init
            pass
        # initialise to initial value, create new d-tree
        self.p_manager.modify_energy(particle_ref=self, set_val=ENERGY_START)
        if merged_tree is not None:
            self.tree = merged_tree
            merged_tree.particle_handler.append(self)
        else:
            self.tree = TreeDisjoint(particle_handler=self)
            self.tree.add_newnode(Node(pos))
            self.tree_manager.disjointedTrees.append(self.tree)
        super().restart(direction, pos)
        return True


class RRdTInformedSampler(ParticleFilterSampler):
    @overrides
    def __init__(self, restart_when_merge=True):
        self.restart_when_merge = restart_when_merge
        super().__init__()

    @overrides
    def init(self, **kwargs):

        super().init(**kwargs)
        self.is_path_found = False
        global MAX_NUMBER_NODES
        MAX_NUMBER_NODES = self.args.max_number_nodes

        self.lsamplers_to_be_restart = []
        self.tree_manager = TreesManager(
            args=self.args, restart_when_merge=self.restart_when_merge)

        # ditch the particles created by the original particle filter sampler, and
        # create ones that has link towards the disjointed tree
        self.p_manager.particles = []
        for _ in range(self.p_manager.num_particles - 1):
            pos = self.p_manager.new_pos_in_free_space()

            dt_p = DisjointTreeParticle(
                direction=random.uniform(0, math.pi * 2),
                pos=pos,
                tree_manager=self.tree_manager,
                p_manager=self.p_manager,
            )

            self.p_manager.particles.append(dt_p)
        # spawn one that comes from the root
        self.p_manager.particles.append(
            DisjointTreeParticle(
                direction=random.uniform(0, math.pi * 2),
                pos=self.start_pos,
                isroot=True,
                startPtNode=self.args.env.startPt,
                tree_manager=self.tree_manager,
                p_manager=self.p_manager,
            ))

        self.cMin = dist(self.start_pos, self.goal_pos) - self.args.goal_radius
        a1 = np.array([[(self.goal_pos[0] - self.start_pos[0]) / self.cMin],
                       [(self.goal_pos[1] - self.start_pos[1]) / self.cMin],
                       [0]])

        self.etheta = math.atan2(a1[1], a1[0])
        self.xCenter = np.array(
            [[(self.start_pos[0] + self.goal_pos[0]) / 2.0],
             [(self.start_pos[1] + self.goal_pos[1]) / 2.0], [0]])
        self.randomSampler = RandomPolicySampler()
        self.randomSampler.init(**kwargs)
        # first column of idenity matrix transposed
        id1_t = np.array([1.0, 0.0, 0.0]).reshape(1, 3)
        M = a1 @ id1_t
        U, S, Vh = np.linalg.svd(M, 1, 1)
        self.C = np.dot(
            np.dot(U, np.diag([1.0, 1.0, np.linalg.det(U) * np.linalg.det(np.transpose(Vh))
                ])), Vh)

    def particles_random_free_space_restart(self):
        for i in range(self.p_manager.size()):
            if self.p_manager.particles_energy[
                    i] < RANDOM_RESTART_PARTICLES_ENERGY_UNDER:
                self.p_manager.add_to_restart(self.p_manager.particles[i])

    @overrides
    def report_success(self, idx, **kwargs):
        self.p_manager.particles[idx].last_node = kwargs['newnode']
        self.p_manager.confirm(idx, kwargs['pos'])
        self.p_manager.modify_energy(idx=idx, factor=0.95)

    @overrides
    def get_valid_next_pos(self):
        """Loop until we find a valid next node"""
        while True:
            _tmp = self.get_next_pos()
            if _tmp is None:
                # This denotes a particle had tried to restart and added the new node
                # to existing tree instead. Skip remaining steps and iterate to next loop
                return None
            rand_pos = _tmp[0]
            
            self.args.env.stats.add_sampled_node(rand_pos)
            if not self.args.env.collides(rand_pos):
                return _tmp
            report_fail = _tmp[-1]
            report_fail(pos=rand_pos, obstacle=True)
            self.args.env.stats.add_invalid(obs=True)

    def restart_all_pending_local_samplers(self):
        # restart all pending local samplers
        while len(self.p_manager.local_samplers_to_be_rstart) > 0:
            # during the proces of restart, if the new restart position
            # is close to an existing tree, it will simply add to that new tree.
            if not self.p_manager.local_samplers_to_be_rstart[0].restart(
                    restart_when_merge=self.restart_when_merge, specific_pos=self.get_random_node_in_ellipse()):
                # This flag denotes that a new position was found among the trees,
                # And it NEEDS to get back to restarting particles in the next ierations
                return False
            self.p_manager.local_samplers_to_be_rstart.pop(0)
        return True

    @staticmethod
    def sampleUnitBall():
        a = random.random()
        b = random.random()
        if b < a:
            a, b = b, a

        sample = (b * math.cos(2 * math.pi * a / b),
                  b * math.sin(2 * math.pi * a / b))
        return np.array([[sample[0]], [sample[1]], [0]])

    @overrides
    def get_next_pos(self):
        self.counter += 1
        self._c_random += 1
        self._c_resample += 1
        self.cBest = self.args.planner.c_max
        if self.args.sampler.cBest < float('inf'):
            self.is_path_found = True
            
        if self._c_random > RANDOM_RESTART_EVERY > 0:
            self._c_random = 0
            self.particles_random_free_space_restart()

        if not self.restart_all_pending_local_samplers():
            LOGGER.debug("Adding node to existing trees.")

            return None
        # get a node to random walk
        choice = self.get_random_choice()

        # NOTE This controls if testing (via mouse) or actual runs
        if self.args.sampler.cBest < float('inf'):
            r = [
                self.cBest / 2.0,
                math.sqrt(self.cBest**2 - self.cMin**2) / 2.0,
                math.sqrt(self.cBest**2 - self.cMin**2) / 2.0
            ]
            L = np.diag(r)
            xBall = self.sampleUnitBall()
            rnd = np.dot(np.dot(self.C, L), xBall) + self.xCenter
            pos = [rnd[(0, 0)], rnd[(1, 0)]]
            print(pos)
        else:
            pos = self.randomWalk(choice)
            #pos = self.randomSampler.get_next_pos()[0]

        #print(pos)
        # pos, choice = self.random_walk_by_mouse()

        return (pos, self.p_manager.particles[choice].tree,
                self.p_manager.particles[choice].last_node,
                lambda c=choice, **kwargs: self.report_success(c, **kwargs),
                lambda c=choice, **kwargs: self.report_fail(c, **kwargs))

    def random_walk_by_mouse(self):
        """FOR testing purpose. Mimic random walk, but do so via mouse click."""
        from planners.mouseSampler import MouseSampler as mouse
        pos = mouse.get_mouse_click_position(scaling=self.scaling)
        # find the cloest particle from this position
        _dist = None
        p_idx = None
        for i in range(len(self.p_manager.particles)):
            p = self.p_manager.particles[i]
            if _dist is None or _dist > dist(pos, p.pos):
                _dist = dist(pos, p.pos)
                p_idx = i
        LOGGER.debug("num of tree: {}".format(
            len(self.tree_manager.disjointedTrees)))
        self.p_manager.new_pos(idx=p_idx, pos=pos, dir=0)
        return pos, p_idx

    @overrides
    def paint(self, window):
        if self._last_prob is None:
            return
        max_num = self._last_prob.max()
        min_num = self._last_prob.min()
        for i, p in enumerate(self.p_manager.particles):
            self.particles_layer.fill((255, 128, 255, 0))
            # get a transition from green to red
            c = self.get_color_transists(self._last_prob[i], max_num, min_num)
            c = max(min(255, c), 50)
            color = (c, c, 0)
            self.args.env.draw_circle(pos=p.pos, colour=color, radius=4, layer=self.particles_layer)
            window.blit(self.particles_layer, (0, 0))
        # draw the ellipse
        if self.args.sampler.cBest < float('inf'):
            cBest = self.cBest * self.args.scaling
            cMin = self.cMin * self.args.scaling
            a = math.sqrt(cBest**2 - cMin**2) # height
            b = cBest # width
            # rectangle that represent the ellipse
            r = pygame.Rect(0, 0, b, a)
            angle = self.etheta
            # rotate via surface

            ellipse_surface = pygame.Surface((b, a), pygame.SRCALPHA, 32).convert_alpha()
            try:
                pygame.draw.ellipse(ellipse_surface, (255,0,0,80), r)
                pygame.draw.ellipse(ellipse_surface, Colour.black, r, int(2 * self.args.scaling))
            except ValueError:
                
                # sometime it will fail to draw due to ellipse being too narrow
                pass
            # rotate
            ellipse_surface = pygame.transform.rotate(ellipse_surface, -angle * 180 / math.pi)
            # we need to offset the blitz based on the surface ceenter
            rcx, rcy = ellipse_surface.get_rect().center
            ellipse_x = (self.xCenter[0] * self.args.scaling - rcx)
            ellipse_y = (self.xCenter[1] * self.args.scaling - rcy)
            try:
                window.blit(ellipse_surface, (int(ellipse_x), int(ellipse_y)))
            except:
                
                # sometime it will fail to draw due to ellipse being too narrow
                print((ellipse_x, ellipse_y))
                raise

    def get_random_node_in_ellipse(self):
        """Return a particle that is in free space (from map)"""
        if not self.is_path_found:
            return None

            
        else:
            while True:
                if self.args.sampler.cBest < float('inf'):
                    r = [
                        self.cBest / 2.0,
                        math.sqrt(self.cBest**2 - self.cMin**2) / 2.0,
                        math.sqrt(self.cBest**2 - self.cMin**2) / 2.0
                    ]
                    L = np.diag(r)
                    xBall = self.sampleUnitBall()
                    rnd = np.dot(np.dot(self.C, L), xBall) + self.xCenter
                    new_p = [rnd[(0, 0)], rnd[(1, 0)]]

                else:
                    new_p = self.randomWalk(choice)
                if not self.args.env.collides(new_p):
                    break
            return new_p

############################################################
##    PATCHING RRT with disjointed-tree specific stuff    ##
############################################################


class Node:
    def __init__(self, pos):
        self.pos = np.array(pos)
        self.cost = 0  # index 0 is x, index 1 is y
        self.edges = []
        self.children = []

    def __repr__(self):
        try:
            num_edges = len(self.edges)
        except AttributeError:
            num_edges = "DELETED"
        return "Node(pos={}, cost={}, num_edges={})".format(
            self.pos, self.cost, num_edges)


class RRdTInformedPlanner(RRTPlanner):

    @overrides
    def init(self, *argv, **kwargs):
        super().init(*argv, **kwargs)
        self.goal_tree_nodes = []
        self.goal_tree_poses = np.empty((self.args.max_number_nodes + 50,
                                         2))  # +50 to prevent over flow
        self.goal_tree_nodes.append(self.args.env.goalPt)
        self.goal_tree_poses[0] = self.args.env.goalPt.pos

        self.found_solution = False
        self.goal_tree_turn = False

    @overrides
    def run_once(self):
        # Get an sample that is free (not in blocked space)
        _tmp = self.args.sampler.get_valid_next_pos()
        if _tmp is None:
            #print(self.args.env.stats.sampledNodes)
            # we have added a new samples when respawning a local sampler
            return
        else:
            pass
            #print('NNNNNNNNNNNNNNNNNNNN')
        rand_pos, parent_tree, last_node, report_success, report_fail = _tmp
        if last_node is not None:
            # use the last succesful node as the nearest node
            # This is expliting the advantage of local sampler :)
            nn = last_node
            newpos = rand_pos
        else:
            idx = self.find_nearest_neighbour_idx(
                rand_pos, parent_tree.poses[:len(parent_tree.nodes)])
            nn = parent_tree.nodes[idx]
            # get an intermediate node according to step-size
            newpos = self.args.env.step_from_to(nn.pos, rand_pos)
        # check if it is free or not ofree
        if not self.args.env.cc.path_is_free(nn.pos, newpos):
            self.args.env.stats.add_invalid(obs=False)
            report_fail(pos=rand_pos, free=False)
        else:
            newnode = Node(newpos)
            self.args.env.stats.add_free()
            self.args.sampler.add_tree_node(newnode.pos)
            report_success(newnode=newnode, pos=newnode.pos)
            ######################
            newnode, nn = self.args.sampler.tree_manager.connect_two_nodes(
                newnode, nn, parent_tree)
            # try to add this newnode to existing trees
            self.args.sampler.tree_manager.add_pos_to_existing_tree(
                newnode, parent_tree)

    def rrt_star_add_node(self, newnode, nn=None):
        """This function perform finding optimal parent, and rewiring."""

        newnode, nn = self.choose_least_cost_parent(
            newnode, nn=nn, nodes=self.args.sampler.tree_manager.root.nodes)
        self.rewire(newnode, nodes=self.args.sampler.tree_manager.root.nodes)
        # check for goal condition
        if dist(newnode.pos, self.goalPt.pos) < self.args.goal_radius:
            if newnode.cost < self.c_max:
                self.c_max = newnode.cost
                self.goalPt.parent = newnode
                newnode.children.append(self.goalPt.parent)
        return newnode, nn


    @overrides
    def paint(self):
        drawn_nodes_pairs = set()
        # Draw disjointed trees
        for tree in self.args.sampler.tree_manager.disjointedTrees:
            bfs = BFS(tree.nodes[0], validNodes=tree.nodes)
            while bfs.has_next():
                newnode = bfs.next()
                for e in newnode.edges:
                    new_set = frozenset({newnode, e})
                    if new_set not in drawn_nodes_pairs:
                        drawn_nodes_pairs.add(new_set)
                        self.args.env.draw_path(newnode, e)
        # Draw root tree
        for n in self.args.sampler.tree_manager.root.nodes:
            if n.parent is not None:
                new_set = frozenset({n, n.parent})
                if new_set not in drawn_nodes_pairs:
                    drawn_nodes_pairs.add(new_set)
                    self.args.env.draw_path(n, n.parent, Colour.orange)

        # for nodes in (self.nodes, self.goal_tree_nodes):
        #     for n in nodes:
        #         if n.parent is not None:
        #             new_set = frozenset({n, n.parent})
        #             if new_set not in drawn_nodes_pairs:
        #                 drawn_nodes_pairs.add(new_set)
        #                 self.args.env.draw_path(n, n.parent)
        self.draw_solution_path()


############################################################
##                         Classes                        ##
############################################################


class TreeRoot:
    def __init__(self, particle_handler):
        self.particle_handler = [particle_handler]
        self.nodes = []
        self.poses = np.empty((MAX_NUMBER_NODES + 50,
                               2))  # +50 to prevent over flow
        # This stores the last node added to this tree (by local sampler)

    def add_newnode(self, node):
        self.poses[len(self.nodes)] = node.pos
        self.nodes.append(node)

    def extend_tree(self, tree):
        self.poses[len(self.nodes):len(self.nodes) +
                   len(tree.nodes)] = tree.poses[:len(tree.nodes)]
        self.nodes.extend(tree.nodes)

    def __repr__(self):
        string = super().__repr__()
        string += '\n'
        import pprint
        string += pprint.pformat(vars(self), indent=4)
        # string += ', '.join("%s: %s" % item for item in vars(self).items())
        return string


class TreeDisjoint(TreeRoot):
    @overrides
    def __init__(self, **kwargs):
        super().__init__(**kwargs)


##########################################################################################
##########################################################################################
##########################################################################################
##########################################################################################
