-- ============================================================
-- Path_Planner.lua
-- Incremental OMPL planning, live planner-data visualization,
-- path export, and planning metrics for CoppeliaSim.
-- ------------------------------------------------------------
-- This script performs geometric motion planning for a drone
-- proxy object using the CoppeliaSim OMPL plugin. The planner
-- is solved incrementally so that the exploration tree/roadmap
-- can be visualized during execution. When an exact solution is
-- found, the path is simplified, interpolated, exported as XYZ
-- waypoints, and shared with the path-following script through
-- CoppeliaSim custom data blocks.
--
-- Main outputs:
--   OMPL_XYZ_PATH       : Interpolated XYZ trajectory.
--   OMPL_PATH_ID        : Incremental identifier for each new path.
--   OMPL_READY          : Handshake flag for the follower script.
--   OMPL_PATH_LENGTH    : Planned path length in meters.
--   OMPL_WAYPOINTS      : Number of interpolated waypoints.
--   OMPL_PLAN_TIME      : Planning time until exact solution.
--   OMPL_ITER_SOLVED    : Iteration at which the solution was found.
-- ============================================================

sim = require 'sim'
simOMPL = require 'simOMPL'

-- Drawing object handles. They are kept globally within the script so that
-- previous drawings can be removed or updated during the same simulation run.
local _plannerDwos = nil      -- OMPL exploration tree/roadmap drawing handle.
local _lineContainer = nil    -- Final trajectory drawing handle.

-- ============================================================
-- Utility functions
-- ============================================================

-- Converts the position of a CoppeliaSim dummy into a 3D pose state.
-- The quaternion is fixed to the identity because only translation is used
-- for geometric path planning in this experiment.
local function pose3dFromDummy(h)
    local p = sim.getObjectPosition(h)
    return {p[1], p[2], p[3], 0, 0, 0, 1}
end

-- Extracts XYZ coordinates from an OMPL pose3d path.
-- OMPL pose3d states are stored as groups of seven values:
-- x, y, z, qx, qy, qz, qw. Only the translational components are used.
local function xyzFromPose3dPath(pathPose3d)
    local xyz = {}
    for i = 1, #pathPose3d, 7 do
        xyz[#xyz + 1] = pathPose3d[i]
        xyz[#xyz + 1] = pathPose3d[i + 1]
        xyz[#xyz + 1] = pathPose3d[i + 2]
    end
    return xyz
end

-- Draws the final interpolated path as connected line segments.
-- Variant parameter:
--   finalPathColor = {0, 1, 1} draws the trajectory in cyan.
--   The color can be changed using normalized RGB values in [0, 1].
local function drawPathXYZ(xyz)
    local finalPathColor = {0, 1, 1}  -- Cyan final path: R=0, G=1, B=1.
    local lineWidth = 3               -- Visual thickness of the final path.
    local maxSegments = 99999         -- Maximum number of drawable segments.

    if not _lineContainer then
        _lineContainer = sim.addDrawingObject(
            sim.drawing_lines,
            lineWidth,
            0,
            -1,
            maxSegments,
            finalPathColor
        )
    end

    -- Clear previous final-path drawing before adding the new one.
    sim.addDrawingObjectItem(_lineContainer, nil)

    -- Add one line segment between each pair of consecutive XYZ waypoints.
    for i = 1, #xyz - 3, 3 do
        sim.addDrawingObjectItem(
            _lineContainer,
            {xyz[i], xyz[i + 1], xyz[i + 2], xyz[i + 3], xyz[i + 4], xyz[i + 5]}
        )
    end
end

-- Computes the Euclidean length of an XYZ path.
local function pathLengthXYZ(xyz)
    local L = 0
    for i = 1, #xyz - 3, 3 do
        local dx = xyz[i + 3] - xyz[i]
        local dy = xyz[i + 4] - xyz[i + 1]
        local dz = xyz[i + 5] - xyz[i + 2]
        L = L + math.sqrt(dx * dx + dy * dy + dz * dz)
    end
    return L
end

-- ============================================================
-- Initialization and handshake
-- ============================================================

function sysCall_init()
    local pathHandle = sim.getObject('/Path')

    -- Clear custom data blocks from previous simulation runs.
    sim.writeCustomDataBlock(pathHandle, 'OMPL_XYZ_PATH', nil)
    sim.writeCustomDataBlock(pathHandle, 'OMPL_PATH_LENGTH', nil)
    sim.writeCustomDataBlock(pathHandle, 'OMPL_WAYPOINTS', nil)
    sim.writeCustomDataBlock(pathHandle, 'OMPL_PLAN_TIME', nil)
    sim.writeCustomDataBlock(pathHandle, 'OMPL_ITER_SOLVED', nil)

    -- Handshake flag used by the path-following script:
    --   0 = no new path is available.
    --   1 = a new path is ready for execution.
    sim.writeCustomDataBlock(pathHandle, 'OMPL_READY', sim.packInt32Table({0}))
end

-- ============================================================
-- OMPL planning routine
-- ============================================================

function sysCall_thread()
    local pathHandle = sim.getObject('/Path')
    local startDummy = sim.getObject('/Path/start')
    local goalDummy  = sim.getObject('/Path/goal')
    local proxy      = sim.getObject('/Path/omplRobot')

    local task = simOMPL.createTask('droneTask')

    -- ------------------------------------------------------------
    -- Planning bounds
    -- ------------------------------------------------------------
    -- Variant parameter:
    --   Modify these limits to match the usable workspace of each scene.
    --   Bounds are expressed as {x, y, z} in meters.
    local low  = {-2.5, -2.5, 0.2}
    local high = { 2.5,  2.5, 1.5}

    -- ------------------------------------------------------------
    -- State-space definition
    -- ------------------------------------------------------------
    -- A pose3d state space is used because OMPL expects 3D position and
    -- orientation. In this experiment, the orientation remains fixed, and
    -- the proxy is used primarily for translational collision checking.
    -- Variant parameter:
    --   The last argument, useForProjection = 1, enables this state space
    --   to be used for projection-based planners such as KPIECE1.
    local ss = {
        simOMPL.createStateSpace(
            's',
            simOMPL.StateSpaceType.pose3d,
            proxy,
            low,
            high,
            1
        )
    }
    simOMPL.setStateSpace(task, ss)

    -- ------------------------------------------------------------
    -- Planner selection
    -- ------------------------------------------------------------
    -- Variant parameter:
    --   Change this line to evaluate different OMPL planners.
    --   Common options include:
    --     simOMPL.Algorithm.RRT
    --     simOMPL.Algorithm.RRTConnect
    --     simOMPL.Algorithm.RRTstar
    --     simOMPL.Algorithm.PRM
    --     simOMPL.Algorithm.PRMLazy
    --     simOMPL.Algorithm.KPIECE1
    --   The exact names may depend on the CoppeliaSim/OMPL version.
    simOMPL.setAlgorithm(task, simOMPL.Algorithm.KPIECE1)

    -- ------------------------------------------------------------
    -- Collision checking
    -- ------------------------------------------------------------
    -- The cubic proxy is checked against all collidable objects in the scene.
    -- This isolates the geometric planning problem from the full dynamic
    -- model of the quadrotor.
    simOMPL.setCollisionPairs(task, {proxy, sim.handle_all})

    -- Variant parameter:
    --   A smaller resolution increases collision-checking precision but also
    --   increases computational cost. A larger value is faster but coarser.
    simOMPL.setStateValidityCheckingResolution(task, 0.02)

    -- Set start and goal states from the corresponding dummy objects.
    simOMPL.setStartState(task, pose3dFromDummy(startDummy))
    simOMPL.setGoalState(task,  pose3dFromDummy(goalDummy))

    simOMPL.setup(task)

    -- ------------------------------------------------------------
    -- Incremental solving and live planner-data visualization
    -- ------------------------------------------------------------
    local maxTime = 20.0       -- Maximum planning budget in seconds.
    local dtSolve = 0.05       -- Time slice for each incremental solve call.
    local maxIters = math.max(1, math.floor(maxTime / dtSolve))

    -- Variant parameter:
    --   drawEvery controls the frequency of live tree/roadmap updates.
    --   Smaller values update more frequently but can reduce performance.
    local drawEvery = 3

    -- Variant parameter:
    --   logEvery controls how often progress information is printed.
    local logEvery = 50

    -- Clear previous OMPL planner-data drawing, if any.
    if _plannerDwos then
        simOMPL.removeDrawingObjects(task, _plannerDwos)
        _plannerDwos = nil
    end

    local t0 = sim.getSystemTimeInMs(-1)
    local solved = false
    local itSolved = 0
    local timeSolvedMs = 0

    for k = 1, maxIters do
        simOMPL.solve(task, dtSolve)

        -- Draw the current exploration tree or roadmap every drawEvery steps.
        if (k % drawEvery) == 0 then
            if _plannerDwos then
                simOMPL.removeDrawingObjects(task, _plannerDwos)
            end

            -- Variant parameters for planner-data visualization:
            --   pointSize        : Size of sampled states.
            --   lineSize         : Thickness of tree/roadmap edges.
            --   edgeColor        : Color of planner branches/roadmap edges.
            --   startStateColor  : Color of the start state.
            --   goalStateColor   : Color of the goal state.
            local pointSize = 0.02
            local lineSize = 1
            local edgeColor = {0.6, 0.6, 0.6}       -- Gray branches/edges.
            local startStateColor = {0, 1, 0}       -- Green start state.
            local goalStateColor = {1, 0, 0}        -- Red goal state.

            _plannerDwos = simOMPL.drawPlannerData(
                task,
                pointSize,
                lineSize,
                edgeColor,
                startStateColor,
                goalStateColor
            )
        end

        -- Optional progress log for debugging and experiment monitoring.
        if (k % logEvery) == 0 then
            local elapsedMs = sim.getSystemTimeInMs(t0)
            sim.addLog(
                sim.verbosity_scriptinfos,
                string.format(
                    'OMPL progress: iter=%d | t=%.2f s | exact=%s | approx=%s',
                    k,
                    elapsedMs / 1000.0,
                    tostring(simOMPL.hasExactSolution(task)),
                    tostring(simOMPL.hasApproximateSolution(task))
                )
            )
        end

        sim.step()

        -- The experiment accepts only exact solutions. Approximate solutions
        -- are ignored to keep the evaluation protocol consistent.
        if simOMPL.hasExactSolution(task) then
            solved = true
            itSolved = k
            timeSolvedMs = sim.getSystemTimeInMs(t0)
            break
        end
    end

    if not solved then
        local elapsedMs = sim.getSystemTimeInMs(t0)
        sim.addLog(
            sim.verbosity_errors,
            string.format('OMPL: No exact path found in %.2f s (iters=%d).', elapsedMs / 1000.0, maxIters)
        )
        return
    end

    -- ------------------------------------------------------------
    -- Path post-processing
    -- ------------------------------------------------------------
    -- simplifyPath reduces unnecessary intermediate states.
    -- interpolatePath enforces a fixed waypoint count for consistent execution.
    -- Variant parameter:
    --   interpolationPoints = 300 defines the number of path states used by
    --   the target-following script.
    simOMPL.simplifyPath(task, -1.0)
    local interpolationPoints = 300
    simOMPL.interpolatePath(task, interpolationPoints)

    local path = simOMPL.getPath(task)
    if not path then
        sim.addLog(sim.verbosity_errors, 'OMPL: Solution reported, but getPath() returned nil.')
        return
    end

    -- Export only XYZ coordinates for the path-following controller.
    local xyz = xyzFromPose3dPath(path)
    sim.writeCustomDataBlock(pathHandle, 'OMPL_XYZ_PATH', sim.packFloatTable(xyz))

    -- Increment PATH_ID only after a valid new path has been generated.
    -- This allows the follower script to detect new trajectories robustly.
    local id = 0
    local packed = sim.readCustomDataBlock(pathHandle, 'OMPL_PATH_ID')
    if packed then
        id = sim.unpackInt32Table(packed)[1]
    end
    id = id + 1
    sim.writeCustomDataBlock(pathHandle, 'OMPL_PATH_ID', sim.packInt32Table({id}))

    -- Mark the path as ready for execution by the path-following script.
    sim.writeCustomDataBlock(pathHandle, 'OMPL_READY', sim.packInt32Table({1}))

    -- Draw the final path using the visualization parameters defined in
    -- drawPathXYZ().
    drawPathXYZ(xyz)

    -- ------------------------------------------------------------
    -- Final planning metrics
    -- ------------------------------------------------------------
    local L = pathLengthXYZ(xyz)
    local nWp = math.floor(#xyz / 3)
    local tPlan = timeSolvedMs / 1000.0

    -- Store metrics so that the path-following script can later print a
    -- complete CSV-compatible experimental record.
    sim.writeCustomDataBlock(pathHandle, 'OMPL_PATH_LENGTH', sim.packFloatTable({L}))
    sim.writeCustomDataBlock(pathHandle, 'OMPL_WAYPOINTS', sim.packInt32Table({nWp}))
    sim.writeCustomDataBlock(pathHandle, 'OMPL_PLAN_TIME', sim.packFloatTable({tPlan}))
    sim.writeCustomDataBlock(pathHandle, 'OMPL_ITER_SOLVED', sim.packInt32Table({itSolved}))

    sim.addLog(
        sim.verbosity_scriptinfos,
        string.format(
            'OMPL PLAN OK: iter_plan=%d | t_plan_s=%.3f | pathLen_m=%.3f | waypoints=%d | PATH_ID=%d',
            itSolved,
            tPlan,
            L,
            nWp,
            id
        )
    )
end
