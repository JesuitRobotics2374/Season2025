/**
 *               ** ** APACHE MAVEN ** **
 * This resource was uploaded to the Apache Maven Repository on 
 * 2025-01-21T02:54:33.000-08:00 by ariesninjadev to 
 * com.ariesninjadev.astar:astar:1.0.0.
 */

package frc.robot.seafinder2.utils;

import com.pathplanner.lib.path.*;
import com.pathplanner.lib.pathfinding.Pathfinder;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.*;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public class AStar implements Pathfinder {

    private static final double corner_smoothing = 0.8; // how much to smooth corners (since we are working on a grid)
    private static final double EPS = 2.5; // "greedyness" (how much should we looked arounf before we move)

    private double fieldLength = 16.54; // meters
    private double fieldWidth = 8.02; // meters

    private double grid_size = 0.2; // size of the A* grid in meters

    private int x_nodes = (int) Math.ceil(fieldLength / grid_size); // calculate the number of nodes
    private int y_nodes = (int) Math.ceil(fieldWidth / grid_size);

    private final HashMap<GridPosition, Double> fm_strt = new HashMap<>(); // track cost from start
    private final HashMap<GridPosition, Double> to_goal = new HashMap<>(); // track cost to goal
    private final HashMap<GridPosition, Pair<Double, Double>> open = new HashMap<>(); // track open nodes
    private final HashMap<GridPosition, Pair<Double, Double>> incons = new HashMap<>(); // not actually needed, but
                                                                                        // would be useful if we ever
                                                                                        // register robots as obstacles.
                                                                                        // for now empty.
    private final Set<GridPosition> closed = new HashSet<>(); // filled nodes
    private final Set<GridPosition> staticObstacles = new HashSet<>(); // static obstacles (physical barriers on the
                                                                       // field; reef, etc.)
    // private final Set<GridPosition> dynamicObstacles = new HashSet<>();
    private final Set<GridPosition> requestObstacles = new HashSet<>(); // if an obstacle is far away, we can discard it
                                                                        // (these are relevant obstacles)

    private GridPosition requestStart; // the requested start position
    private Translation2d requestRealStartPos; // the requested start position in field coordinates with 0,0 at the
                                               // center of the field
    private GridPosition requestGoal;
    private Translation2d requestRealGoalPos;

    private double eps; // in an effort to make the algorithm faster, we can be less greedy (look around
                        // less) if we are far away from the goal. we use some calculus I honestly dont
                        // understand to calculate this

    private final Thread planningThread; // WPILib mandates that we do asynchronous operations in a new thread (since we
                                         // use timed skeleton and we can't overrun the periodic)
    private boolean requestMinor = true; // keep track of what recalculation should be done based on the requests
    private boolean requestMajor = true;
    private boolean requestReset = true; // reset the algorithm?
    private boolean newPathAvailable = false; // if a new path is available (which we can use to determine if we should
                                              // recalculate the path)

    private final ReadWriteLock pathLock = new ReentrantReadWriteLock(); // lock for the path. I honestly would have
                                                                         // given up on this project if I didn't find
                                                                         // this.
    private final ReadWriteLock requestLock = new ReentrantReadWriteLock(); // lock for the requests

    private List<Waypoint> currentWaypoints = new ArrayList<>(); // the current path as a list of waypoints( WAYYYYY
                                                                 // faster to sift through in op block )
    private List<GridPosition> currentPathFull = new ArrayList<>(); // the current path as a list of grid positions

    public AStar() {
        planningThread = new Thread(this::runThread); // create the thread

        requestStart = new GridPosition(0, 0); // initialize the request
        requestRealStartPos = Translation2d.kZero;
        requestGoal = new GridPosition(0, 0); // and the goal
        requestRealGoalPos = Translation2d.kZero;

        staticObstacles.clear(); // clear the obstacles
        // dynamicObstacles.clear();

        File navGridFile = new File(Filesystem.getDeployDirectory(), "pathplanner/navgrid.json"); // load our array
                                                                                                  // of obstacles from
                                                                                                  // the navgrid file

        // all this code is to parse the navgrid file
        if (navGridFile.exists()) {
            try (BufferedReader br = new BufferedReader(new FileReader(navGridFile))) {
                StringBuilder fileContentBuilder = new StringBuilder();
                String line;
                while ((line = br.readLine()) != null) {
                    fileContentBuilder.append(line);
                }

                String fileContent = fileContentBuilder.toString();
                JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

                grid_size = ((Number) json.get("nodeSizeMeters")).doubleValue();
                JSONArray grid = (JSONArray) json.get("grid");
                y_nodes = grid.size();
                for (int row = 0; row < grid.size(); row++) {
                    JSONArray rowArray = (JSONArray) grid.get(row);
                    if (row == 0) {
                        x_nodes = rowArray.size();
                    }
                    for (int col = 0; col < rowArray.size(); col++) {
                        boolean isObstacle = (boolean) rowArray.get(col);
                        if (isObstacle) {
                            staticObstacles.add(new GridPosition(col, row));
                        }
                    }
                }

                JSONObject fieldSize = (JSONObject) json.get("field_size");
                fieldLength = ((Number) fieldSize.get("x")).doubleValue();
                fieldWidth = ((Number) fieldSize.get("y")).doubleValue();
            } catch (Exception e) {
            }
        }

        // add all the obstacles to the request
        requestObstacles.clear();
        requestObstacles.addAll(staticObstacles);
        // requestObstacles.addAll(dynamicObstacles);

        // init
        requestReset = true;
        requestMajor = true;
        requestMinor = true;
        newPathAvailable = false;

        planningThread.setDaemon(true); // never got the chance to test, but this should prevent the thread from running
                                        // after the robot is disabled and re-enabled (that would probably be bad)
        planningThread.setName("Aries' goofy A* thread"); // i don't know why I did this
        planningThread.start(); // run the thread
    }

    // all overrides are mandatory, since the autobuilder will request this data
    // when using pathplannerlib splines and other nice features)

    @Override
    public boolean isNewPathAvailable() {
        return newPathAvailable; // if a new path is available
    }

    // if we want to disable anytime (since it may cause scary amounts of jerk, as
    // i'm too lazy to do transition splines), return the initial path instead of
    // the current path

    @Override
    public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
        List<Waypoint> waypoints;

        pathLock.readLock().lock(); // lock the path before transmitting. this prevents calculation stripping
                                    // (copyright aries powvalla)
        waypoints = new ArrayList<>(currentWaypoints); // copy the path
        pathLock.readLock().unlock(); // unlock the path

        newPathAvailable = false; // since we just released a path to PPL, we can assume that the path is no
                                  // longer new

        if (waypoints.size() < 2) {
            return null; // i spent 3 hours on this, but for some reason we can end up with a
                         // path of size 1. this is a workaround for that. a path with size 1 is the
                         // equivalent of a single point, so we would already be flagging as ended
                         // anyways. I guess java daemons don't like me
        }

        return new PathPlannerPath(waypoints, constraints, null, goalEndState); // return the path
    }

    // set a new start position, after we make sure we aren't in a wall
    // algorithmically, since A* kinda breaks if we start in a wall
    @Override
    public void setStartPosition(Translation2d startPosition) {
        GridPosition startPos = findClosestNonObstacle(getGridPos(startPosition), requestObstacles);

        if (startPos != null && !startPos.equals(requestStart)) { // if the start position is not null and not the same
                                                                  // as the current start position
            requestLock.writeLock().lock(); // do our locky lock
            requestStart = startPos; // set the start position
            requestRealStartPos = startPosition; // set the cartesian start position

            requestMinor = true; // we will request a minor recalculation (behind the furthest scan)
            requestLock.writeLock().unlock(); // unlock
        }
    }

    // exact same as above, but for the goal
    @Override
    public void setGoalPosition(Translation2d goalPosition) {
        GridPosition gridPos = findClosestNonObstacle(getGridPos(goalPosition), requestObstacles);

        if (gridPos != null) {
            requestLock.writeLock().lock();
            requestGoal = gridPos;
            requestRealGoalPos = goalPosition;

            requestMinor = true;
            requestMajor = true;
            requestReset = true;
            requestLock.writeLock().unlock();
        }
    }

    // the primary execution job
    @SuppressWarnings("BusyWait") // i don't know how to fix this lowk
    private void runThread() {
        while (true) {
            try {
                // set the request variables
                requestLock.readLock().lock();
                boolean reset = requestReset;
                boolean minor = requestMinor;
                boolean major = requestMajor;
                GridPosition start = requestStart;
                Translation2d realStart = requestRealStartPos;
                GridPosition goal = requestGoal;
                Translation2d realGoal = requestRealGoalPos;
                Set<GridPosition> obstacles = new HashSet<>(requestObstacles);

                // sync external stored variables with the master daemon
                if (reset) {
                    requestReset = false;
                }

                if (minor) {
                    requestMinor = false;
                } else if (major && (eps - 0.5) <= 1.0) {
                    requestMajor = false;
                }
                requestLock.readLock().unlock(); // *turns key*

                if (reset || minor || major) {
                    timeForBigBoyMath(reset, minor, major, start, goal, realStart, realGoal, obstacles); // you can see
                                                                                                         // my code
                                                                                                         // deteriorating
                } else {
                    try {
                        Thread.sleep(10); // sync at double the speed of the periodic (any faster and the rio starts
                                          // hanging... it still hangs around once per 5 secs but i don't care)
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }
            } catch (Exception e) {
                requestLock.writeLock().lock(); // if something goes wrong, we can assume that the thread is goofed and
                                                // we need to reset
                requestReset = true;
                requestLock.writeLock().unlock();
            }
        }
    }

    private void timeForBigBoyMath(
            boolean needsReset,
            boolean doMinor,
            boolean doMajor,
            GridPosition sStart,
            GridPosition sGoal,
            Translation2d realStartPos,
            Translation2d realGoalPos,
            Set<GridPosition> obstacles) {
        if (needsReset) { // reset the algorithm
            reset(sStart, sGoal);
        }

        if (doMinor) { // I should probably have used an enum since there are two seperate
                       // recalculation types that each have a variable, but I'll fix this later
            improveStuff(sStart, sGoal, obstacles);

            List<GridPosition> pathPositions = extractPath(sStart, sGoal, obstacles); // reference the code below
            List<Waypoint> waypoints = createWaypoints(pathPositions, realStartPos, realGoalPos, obstacles);

            pathLock.writeLock().lock(); // update caches
            currentPathFull = pathPositions;
            currentWaypoints = waypoints;
            pathLock.writeLock().unlock();

            newPathAvailable = true; // we have a new path, and we can always set this back to false if we need to
                                     // (done above)
        } else if (doMajor) { // and this is when he knew: he messed up
            if (eps > 1.0) { // reminder that eps is the greedyness variable. we only need to actually do a
                             // major recalculation if we've seen enough of the field, otherwise it's the
                             // equivalent of starting from scratch
                eps -= 0.5;
                open.putAll(incons); // put all the inconsistencies into the open list, including nodes that were
                                     // originally pathed but were then removed after optimization

                open.replaceAll((s, v) -> key(s, sStart)); // replace all the keys with the new keys
                closed.clear(); // clear the closed list
                improveStuff(sStart, sGoal, obstacles); // now run the recalculation

                List<GridPosition> pathPositions = extractPath(sStart, sGoal, obstacles);
                List<Waypoint> waypoints = createWaypoints(pathPositions, realStartPos, realGoalPos, obstacles);

                pathLock.writeLock().lock();
                currentPathFull = pathPositions;
                currentWaypoints = waypoints;
                pathLock.writeLock().unlock();

                newPathAvailable = true;
            }
        }
    }

    // A* algorithm. Uses Dijkstra's algorithm to find the shortest path from the
    // start to the goal, and then adjusts h(v) to comply with the greedyness (EPS)
    private List<GridPosition> extractPath(
            GridPosition sStart, GridPosition sGoal, Set<GridPosition> obstacles) {
        if (sGoal.equals(sStart)) { // if the start and goal are the same, we don't need to do anything (YIPPEEE!)
            return new ArrayList<>();
        }

        List<GridPosition> path = new ArrayList<>(); // start a new path
        path.add(sStart); // add the start position

        var s = sStart; // set the current position to the start position

        for (int k = 0; k < 200; k++) { // we don't want to run forever, so we set a limit of 200 iterations. We also
                                        // lower the eps by 0.5 every iteration, so we can assume that the path will be
                                        // found in 200 iterations
            HashMap<GridPosition, Double> gList = new HashMap<>(); // create a new list of g values

            // for each open neighbor, add the g value to the running list
            for (GridPosition x : getOpenNeighbors(s, obstacles)) {
                gList.put(x, fm_strt.get(x));
            }

            Map.Entry<GridPosition, Double> min = Map.entry(sGoal, Double.POSITIVE_INFINITY); // set the minimum to the
                                                                                              // goal position, and the
                                                                                              // value to infinity
                                                                                              // (since we are looking
                                                                                              // for the minimum)
            // for each entry in the g list, find the minimum value
            for (var entry : gList.entrySet()) {
                if (entry.getValue() < min.getValue()) {
                    min = entry;
                }
            }
            s = min.getKey(); // set the current position to the minimum

            path.add(s); // add the current position to the path
            if (s.equals(sGoal)) { // if the current position is the goal, we have found our entire path!
                break;
            }
        }

        return path; // return the path
    }

    // convert a path from a list of grid positions to a list of waypoints that we
    // can supply to PPL. some of this code is heavily inspired.
    private List<Waypoint> createWaypoints(
            List<GridPosition> path,
            Translation2d realStartPos,
            Translation2d realGoalPos,
            Set<GridPosition> obstacles) {
        if (path.isEmpty()) { // if the path is empty, we don't need to do anything. this should never
                              // happen! :D (honestly we should probably kill the thread if this happens)
            return new ArrayList<>();
        }

        List<GridPosition> simplifiedPath = new ArrayList<>(); // track our simplified path
        simplifiedPath.add(path.get(0)); // add the start position
        // for each position in the path, check if the line between the last position in
        // the simplified path and the current position is "walkable", meaning we can
        // draw a line between the two without hitting an obstacle. this is my favorite
        // piece of code because it's like that final piece of the puzzle that clicks
        // between a grid of points to a smooth path for the robot to follow.
        for (int i = 1; i < path.size() - 1; i++) {
            if (!walkable(simplifiedPath.get(simplifiedPath.size() - 1), path.get(i + 1), obstacles)) {
                simplifiedPath.add(path.get(i)); // add the position to the simplified path
            }
        }
        simplifiedPath.add(path.get(path.size() - 1)); // add the goal position

        // track our path in field coordinates
        List<Translation2d> fieldPosPath = new ArrayList<>();
        for (GridPosition pos : simplifiedPath) {
            fieldPosPath.add(gridPosToTranslation2d(pos));
        }

        if (fieldPosPath.size() < 2) { // this is what i was talking about earlier. WHAT THE FRIDGE? this is a hacky fix
                                       // and should be removed later.
            return new ArrayList<>();
        }

        fieldPosPath.set(0, realStartPos); // add the start point
        fieldPosPath.set(fieldPosPath.size() - 1, realGoalPos); // add the end point

        List<Pose2d> pathPoses = new ArrayList<>(); // convert points to Pose2d's
        pathPoses.add(
                new Pose2d(fieldPosPath.get(0), fieldPosPath.get(1).minus(fieldPosPath.get(0)).getAngle()));

        // for each point after the starting point, create two poses, each at a splined
        // angle matching standard ax+n to smooth rotational acceleration (on swerve
        // only)
        for (int i = 1; i < fieldPosPath.size() - 1; i++) {
            Translation2d last = fieldPosPath.get(i - 1);
            Translation2d current = fieldPosPath.get(i);
            Translation2d next = fieldPosPath.get(i + 1);

            Translation2d anchor1 = current.minus(last).times(corner_smoothing).plus(last);
            Rotation2d heading1 = current.minus(last).getAngle();
            Translation2d anchor2 = current.minus(next).times(corner_smoothing).plus(next);
            Rotation2d heading2 = next.minus(anchor2).getAngle();

            pathPoses.add(new Pose2d(anchor1, heading1));
            pathPoses.add(new Pose2d(anchor2, heading2));
        }

        // add the last pose. we try to compensate if the end point is in a wall.
        pathPoses.add(
                new Pose2d(
                        fieldPosPath.get(fieldPosPath.size() - 1),
                        fieldPosPath
                                .get(fieldPosPath.size() - 1)
                                .minus(fieldPosPath.get(fieldPosPath.size() - 2))
                                .getAngle()));

        return PathPlannerPath.waypointsFromPoses(pathPoses); // return the waypoints
    }

    private GridPosition findClosestNonObstacle(GridPosition pos, Set<GridPosition> obstacles) {
        if (!obstacles.contains(pos)) {
            return pos; // if the position is not an obstacle, we don't need to do anything
        }

        Set<GridPosition> visited = new HashSet<>(); // track visited positions

        Queue<GridPosition> queue = new LinkedList<>(getAllNeighbors(pos)); // add all the neighbors to the queue

        while (!queue.isEmpty()) { // while the queue is not empty
            GridPosition check = queue.poll(); // get the next position
            if (!obstacles.contains(check)) { // if the position is not an obstacle, we have a valid point
                return check; // return the position
            }
            visited.add(check); // else, add the position to the visited set

            for (GridPosition neighbor : getAllNeighbors(check)) { // for each neighbor of the position
                if (!visited.contains(neighbor) && !queue.contains(neighbor)) { // if the neighbor is not visited and
                                                                                // not in the queue, add it to the
                                                                                // queue
                    queue.add(neighbor);
                }
            }
        }
        return null;
    }

    // reset the algorithm
    private void reset(GridPosition sStart, GridPosition sGoal) {
        // clear all the lists
        fm_strt.clear();
        to_goal.clear();
        open.clear();
        incons.clear();
        closed.clear();

        // add all the nodes to the lists
        for (int x = 0; x < x_nodes; x++) {
            for (int y = 0; y < y_nodes; y++) {
                fm_strt.put(new GridPosition(x, y), Double.POSITIVE_INFINITY);
                to_goal.put(new GridPosition(x, y), Double.POSITIVE_INFINITY);
            }
        }

        to_goal.put(sGoal, 0.0); // set the goal cost to 0

        eps = EPS; // set the eps to the default value

        open.put(sGoal, key(sGoal, sStart)); // add the goal to the open list
    }

    // checks if a line between two points is unobstructed
    private boolean walkable(GridPosition s1, GridPosition s2, Set<GridPosition> obstacles) {

        // this is basically walking the slope and accounting for error caused by the
        // fact that we are on a grid

        int x0 = s1.x;
        int y0 = s1.y;
        int x1 = s2.x;
        int y1 = s2.y;

        int change_x = Math.abs(x1 - x0); // calculate the change in x and y
        int change_y = Math.abs(y1 - y0);

        int x = x0; // set the current position to the start position
        int y = y0;

        int n = 2 + change_x + change_y - 1; // calculate the number of iterations

        int xInc = (x1 > x0) ? 1 : -1; // calculate the increment in x and y
        int yInc = (y1 > y0) ? 1 : -1;

        int error = change_x - change_y; // calculate the error
        change_x *= 2; // double the change in x and y
        change_y *= 2;

        for (int i = 0; i < n; i++) { // iterate through the line
            if (obstacles.contains(new GridPosition(x, y))) {
                return false; // if the position is an obstacle, return false
            }

            // account for error
            if (error > 0) {
                x += xInc;
                error -= change_y;
            } else if (error < 0) {
                y += yInc;
                error += change_x;
            } else {
                x += xInc;
                y += yInc;
                error -= change_y;
                error += change_x;
                i++; // Skip next iteration since we moved diagonally
            }
        }

        return true;
    }

    // improve the pathing "anytime" (behind the furthest scan)
    private void improveStuff(
            GridPosition sStart, GridPosition sGoal, Set<GridPosition> obstacles) {
        for (int k = 0; k < 10000000; k++) { // technically we should never get stuck here, but crashing this thread
                                             // would be REALLY bad and might even crash the robot so lets be safe
            var sv = topKey(); // get the nearest relevant node within the scan bound
            if (sv == null) { // if there are no more nodes, we are done
                break;
            }
            var s = sv.getFirst(); // get the node
            var v = sv.getSecond(); // get the key

            if (comparePair(v, key(sStart, sStart)) >= 0 && to_goal.get(sStart).equals(fm_strt.get(sStart))) { // if the
                                                                                                               // key is
                                                                                                               // greater
                                                                                                               // than
                                                                                                               // the
                                                                                                               // start
                                                                                                               // key,
                                                                                                               // we are
                                                                                                               // done
                                                                                                               // (or
                                                                                                               // we'd
                                                                                                               // be
                                                                                                               // moving
                                                                                                               // in
                                                                                                               // circles)
                break;
            }

            open.remove(s); // remove the node from the open list

            if (fm_strt.get(s) > to_goal.get(s)) { // if the cost from start is greater than the cost to goal, we need
                                                   // to update the node regardless of the closed list
                fm_strt.put(s, to_goal.get(s)); // update the cost from start
                closed.add(s); // add the node to the closed list

                for (GridPosition sn : getOpenNeighbors(s, obstacles)) { // for each neighbor of the node
                    updateState(sn, sStart, sGoal, obstacles); // update the neighbor
                }
            } else {
                fm_strt.put(s, Double.POSITIVE_INFINITY); // set the cost from start to infinity (we are going to
                                                          // recalculate it, shifting down)
                for (GridPosition sn : getOpenNeighbors(s, obstacles)) {
                    updateState(sn, sStart, sGoal, obstacles);
                }
                updateState(s, sStart, sGoal, obstacles); // update the node a final time
            }
        }
    }

    // helps replan a path around an outermost point
    private void updateState(
            GridPosition s, GridPosition sStart, GridPosition sGoal, Set<GridPosition> obstacles) {
        if (!s.equals(sGoal)) { // if the node is not the goal, we have to update the cost to goal
            to_goal.put(s, Double.POSITIVE_INFINITY);

            // for each neighbor of the node, update the cost to goal (diverse check)
            for (GridPosition x : getOpenNeighbors(s, obstacles)) {
                to_goal.put(s, Math.min(to_goal.get(s), fm_strt.get(x) + cost(s, x, obstacles))); // update the cost to
                                                                                                  // goal by finding the
                                                                                                  // minimum cost to
                                                                                                  // goal among the
                                                                                                  // neighbors
            }
        }

        open.remove(s); // remove the node from the open list

        // if the cost from start is greater than the cost to goal, we need to update
        // the node
        if (!fm_strt.get(s).equals(to_goal.get(s))) {
            if (!closed.contains(s)) { // if the node is not in the closed list, we need to add it to the open list
                open.put(s, key(s, sStart));
            } else { // the node is inconsistent, so we need to add it to mark it as viable for
                     // recalculation
                incons.put(s, Pair.of(0.0, 0.0));
            }
        }
    }

    // a neighbor is succeptible to being a wall when scanned by the state updater,
    // so we use this wrapper
    private double cost(GridPosition sStart, GridPosition sGoal, Set<GridPosition> obstacles) {
        if (isCollision(sStart, sGoal, obstacles)) {
            return Double.POSITIVE_INFINITY; // if there is a collision, the cost is infinite (unusable)
        }

        return trueCostFinder(sStart, sGoal); // perform cost analysis
    }

    // very similar to the walkable function, but we also check if the line is a
    // diagonal
    private boolean isCollision(GridPosition sStart, GridPosition sEnd, Set<GridPosition> obstacles) {
        if (obstacles.contains(sStart) || obstacles.contains(sEnd)) {
            return true; // if the start or end is an obstacle, there is a collision (lol)
        }

        // if the line is diagonal, we need to check if the line intersects with any
        // obstacles
        if (sStart.x != sEnd.x && sStart.y != sEnd.y) { // if the line is diagonal
            GridPosition s1;
            GridPosition s2;

            // determine the two points that the line intersects with on the perpendicular
            // diagonal. maybe i should make this more efficient
            if (sEnd.x - sStart.x == sStart.y - sEnd.y) {
                s1 = new GridPosition(Math.min(sStart.x, sEnd.x), Math.min(sStart.y, sEnd.y));
                s2 = new GridPosition(Math.max(sStart.x, sEnd.x), Math.max(sStart.y, sEnd.y));
            } else {
                s1 = new GridPosition(Math.min(sStart.x, sEnd.x), Math.max(sStart.y, sEnd.y));
                s2 = new GridPosition(Math.max(sStart.x, sEnd.x), Math.min(sStart.y, sEnd.y));
            }

            return obstacles.contains(s1) || obstacles.contains(s2);
        }

        return false;
    }

    // get the neighbors of a node that are not obstacles
    private List<GridPosition> getOpenNeighbors(GridPosition s, Set<GridPosition> obstacles) {
        List<GridPosition> ret = new ArrayList<>(); // track the neighbors

        // for each neighbor of the node, check if it is an obstacle
        for (int xMove = -1; xMove <= 1; xMove++) {
            for (int yMove = -1; yMove <= 1; yMove++) {
                GridPosition sNext = new GridPosition(s.x + xMove, s.y + yMove); // get the neighbor
                if (!obstacles.contains(sNext)
                        && sNext.x >= 0
                        && sNext.x < x_nodes
                        && sNext.y >= 0
                        && sNext.y < y_nodes) {
                    ret.add(sNext); // add the neighbor to the list
                }
            }
        }
        return ret;
    }

    // get all the neighbors of a node, including obstacles
    private List<GridPosition> getAllNeighbors(GridPosition s) {
        List<GridPosition> ret = new ArrayList<>();

        // for each neighbor of the node, check if it is within the bounds of the grid
        for (int xMove = -1; xMove <= 1; xMove++) {
            for (int yMove = -1; yMove <= 1; yMove++) {
                GridPosition sNext = new GridPosition(s.x + xMove, s.y + yMove);
                if (sNext.x >= 0 && sNext.x < x_nodes && sNext.y >= 0 && sNext.y < y_nodes) {
                    ret.add(sNext); // add the neighbor to the list
                }
            }
        }
        return ret;
    }

    // get the key of a node (decoder)
    private Pair<Double, Double> key(GridPosition s, GridPosition sStart) {
        if (fm_strt.get(s) > to_goal.get(s)) { // if the cost from start is greater than the cost to goal, we warn the
                                               // anytime system by drastically increasing the cost to goal
            return Pair.of(to_goal.get(s) + eps * trueCostFinder(sStart, s), to_goal.get(s));
        } else { // otherwise, we can just use the cost from start
            return Pair.of(fm_strt.get(s) + trueCostFinder(sStart, s), fm_strt.get(s));
        }
    }

    // get the top key of the open list
    private Pair<GridPosition, Pair<Double, Double>> topKey() {
        Map.Entry<GridPosition, Pair<Double, Double>> min = null; // track the minimum
        for (var entry : open.entrySet()) {
            if (min == null || comparePair(entry.getValue(), min.getValue()) < 0) { // if the entry is less than the
                                                                                    // minimum, we update the minimum
                min = entry;
            }
        }

        if (min == null) {
            return null; // if there is no minimum, we are done
        }

        return Pair.of(min.getKey(), min.getValue()); // return the minimum
    }

    // this code used to be 200+ lines long, and was this intricate plan to have
    // super precise cost estimates. I spent hours and many class periods working on
    // it, all to realize that I could stay within 10% error and cut the computing
    // cost by 500% by just finding the hypotenuse.
    private double trueCostFinder(GridPosition sStart, GridPosition sGoal) {
        return Math.hypot(sGoal.x - sStart.x, sGoal.y - sStart.y);
    }

    // compare two pairs
    private int comparePair(Pair<Double, Double> a, Pair<Double, Double> b) {
        int first = Double.compare(a.getFirst(), b.getFirst());
        if (first == 0) {
            return Double.compare(a.getSecond(), b.getSecond());
        } else {
            return first;
        }
    }

    // convert a position in field coordinates to a position on the grid (encoder)
    private GridPosition getGridPos(Translation2d pos) {
        int x = (int) Math.floor(pos.getX() / grid_size);
        int y = (int) Math.floor(pos.getY() / grid_size);

        return new GridPosition(x, y);
    }

    // convert a position on the grid to a position in field coordinates (decoder)
    private Translation2d gridPosToTranslation2d(GridPosition pos) {
        return new Translation2d(
                (pos.x * grid_size) + (grid_size / 2.0), (pos.y * grid_size) + (grid_size / 2.0));
    }

    // this is a record that holds the position of a node on the grid. it allows us
    // to compare an arbitrary datatype EXTREMELY quickly.
    public record GridPosition(int x, int y) implements Comparable<GridPosition> {
        @Override
        public int compareTo(GridPosition o) {
            if (x == o.x) {
                return Integer.compare(y, o.y);
            } else {
                return Integer.compare(x, o.x);
            }
        }
    }

    // we dont need this POOP code (im not smart enough)
    @Override
    public void setDynamicObstacles(List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
        throw new UnsupportedOperationException("Unimplemented method 'setDynamicObstacles'");
    }
}