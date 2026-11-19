-- Eddie spawner + driver: spawns etarak\props\eddie\eddie.ent and drives the newest one around.
-- Needs only Cyber Engine Tweaks (CET's built-in exEntitySpawner + vanilla game systems).
--
-- CET overlay -> "Eddie" window: Spawn / Despawn, drive modes, speed.
-- Bind keys in the CET overlay under Bindings -> eddie_spawner:
--   hotkeys: spawn, despawn, follow V on/off, send to where V looks, stop
--   inputs (hold): forward / back / strafe left / strafe right / turn left / turn right
--
-- The base is holonomic like the real KELO drives: it can strafe and turn in place.
-- Movement is kinematic (the entity is moved every frame). The drive units swivel like
-- shopping-cart castors (wheels trailing their pivot) and the wheels roll with the
-- ground speed: eddie_parts.lua (written by install_wolvenkit.py) has the layout, and the
-- eddie_part_* mesh components in eddie.ent are moved with SetLocalTransform.
--
-- Riding: "Ride" (or the eddie_ride hotkey) puts V on top of Eddie's torso. V's own walking
-- is switched off with the GameplayRestriction.NoMovement status effect and V is moved with
-- the robot every frame; W/A/S/D (or the left stick) drive Eddie relative to where V looks,
-- Eddie turns to face V's view, and Jump gets off.
--
-- Arms: Rest / Reach / Arms up / Wave and grippers open / close. Every arm link and gripper
-- part is its own eddie_part_* mesh component, placed each frame from the joint values
-- (forward kinematics over eddie_parts.lua -> arms.links); the poses come from build_glb.py.

local ENT_PATH = "etarak\\props\\eddie\\eddie.ent"
local RIDE_SPOT = { 0.0, -0.185, 0.93 } -- m in Eddie's frame (right, forward, up): top of the torso
local RIDE_REACH = 10.0     -- m, how close V has to be to get on
local RIDE_RESTRICTION = "GameplayRestriction.NoMovement"
local RIDE_ACTIONS = { "Forward", "Back", "Left", "Right", "Jump" }
local STEER_RATE_MAX = 8.0  -- rad/s, fastest castor swivel (a 180 deg flip takes ~0.4 s)
local SPAWN_DISTANCE = 2.0  -- m in front of V
local FOLLOW_DISTANCE = 2.0 -- m, keeps this far from V
local ACCEL = 1.5           -- m/s^2
local TURN_RATE = 120       -- deg/s
local MAX_STEP = 0.04       -- m, highest bump it drives over (KELO wheels don't climb curbs)
local BODY_RADIUS = 0.55    -- m, centre to the outermost collision box corner (0.52)
local GROUND_GROUPS = { "Static", "Terrain" }

local robots = {}           -- { id, pos = {x,y,z}, yaw, vel = {x,y} }, last one is driven
local mode = "idle"         -- idle | follow | goto | manual | ride
local goal = nil            -- {x,y} for goto
local speed = 1.0           -- m/s
local keys = { fwd = false, back = false, left = false, right = false, tleft = false, tright = false }
local yawSign = -1          -- game yaw convention, calibrated from V on first spawn
local overlayOpen = false
local status = ""
local riding = false
-- V's movement input while riding (0..1 each), mouse yaw collected since the last frame (deg),
-- right-stick yaw rate (deg per 60 Hz frame), V's view yaw, a pending "get off"
local ride = { fwd = 0, back = 0, left = 0, right = 0, mouse = 0, stick = 0, yaw = 0, getOff = false,
               mouseSens = 1, padSens = 1 }

local function say(msg)
    status = msg
    print("[eddie_spawner] " .. msg)
end

local hasParts, PARTS = pcall(require, "eddie_parts")
if not hasParts or type(PARTS) ~= "table" then PARTS = nil end
local atan2 = math.atan2 or math.atan

-- yaw (deg) <-> horizontal direction, in the game's convention
local function dirFromYaw(yaw)
    local r = math.rad(yaw)
    return yawSign * math.sin(r), math.cos(r)
end

local function yawFromDir(dx, dy)
    return math.deg((math.atan2 or math.atan)(yawSign * dx, dy))
end

local function wrap(a)
    while a > 180 do a = a - 360 end
    while a < -180 do a = a + 360 end
    return a
end

local function calibrate(player)
    local f = player:GetWorldForward()
    local r = math.rad(player:GetWorldYaw())
    -- forward = (s*sin(yaw), cos(yaw)); pick the s that matches V
    yawSign = (f.x * math.sin(r) >= 0) and 1 or -1
end

local function raycast(from, to)
    local sqs = Game.GetSpatialQueriesSystem()
    local best = nil
    for _, group in ipairs(GROUND_GROUPS) do
        local ok, hit = sqs:SyncRaycastByCollisionGroup(from, to, group, false, false)
        if ok and hit then
            local p = hit.position
            local d = (p.x - from.x) ^ 2 + (p.y - from.y) ^ 2 + (p.z - from.z) ^ 2
            if not best or d < best.d then best = { x = p.x, y = p.y, z = p.z, d = d } end
        end
    end
    return best
end

-- Floor height under (x, y) near z, or nil for a drop or a step too high to drive up.
-- The probe starts just above the floor under the robot's centre, which is free space
-- (the chassis box starts 6 cm up and the drive units sit at the corners), so it can't
-- hit Eddie's own colliders.
local function groundAt(x, y, z)
    local top = z + MAX_STEP + 0.005
    local hit = raycast(Vector4.new(x, y, top, 1), Vector4.new(x, y, z - 1.0, 1))
    if not hit or hit.z > top - 0.004 or hit.z < z - 0.6 then
        return nil
    end
    return hit.z
end

-- wall check in the driving direction, starting outside Eddie's own collision boxes
local function blocked(r, dx, dy)
    local len = math.sqrt(dx * dx + dy * dy)
    if len < 1e-6 then return false end
    dx, dy = dx / len, dy / len
    local z = r.pos.z + 0.2
    local from = Vector4.new(r.pos.x + dx * BODY_RADIUS, r.pos.y + dy * BODY_RADIUS, z, 1)
    local to = Vector4.new(r.pos.x + dx * (BODY_RADIUS + 0.25), r.pos.y + dy * (BODY_RADIUS + 0.25), z, 1)
    return raycast(from, to) ~= nil
end

local function active()
    return robots[#robots]
end

-- ---------------------------------------------------------------------------
-- Riding
-- ---------------------------------------------------------------------------
local function setRestriction(on)
    local p = Game.GetPlayer()
    if not p then return end
    local ses, id = Game.GetStatusEffectSystem(), p:GetEntityID()
    if on then
        ses:ApplyStatusEffect(id, RIDE_RESTRICTION, p:GetRecordID(), id)
    else
        ses:RemoveStatusEffect(id, RIDE_RESTRICTION)
    end
end

-- a floor spot next to Eddie to step off onto, with no wall in between: right, left, back, front
local function stepOffSpot(r)
    local fx, fy = dirFromYaw(r.yaw)
    local sides = { { fy, -fx }, { -fy, fx }, { -fx, -fy }, { fx, fy } }
    local d, z = BODY_RADIUS + 0.45, r.pos.z
    for _, o in ipairs(sides) do
        local x, y = r.pos.x + o[1] * d, r.pos.y + o[2] * d
        local wall = raycast(Vector4.new(r.pos.x + o[1] * BODY_RADIUS, r.pos.y + o[2] * BODY_RADIUS, z + 0.5, 1),
            Vector4.new(x, y, z + 0.5, 1))
        local floor = raycast(Vector4.new(x, y, z + 0.5, 1), Vector4.new(x, y, z - 1.0, 1))
        if not wall and floor then return x, y, floor.z end
    end
    return r.pos.x + sides[1][1] * d, r.pos.y + sides[1][2] * d, z
end

local function stopRiding(msg)
    if not riding then return end
    riding, ride.getOff = false, false
    if mode == "ride" then mode = "idle" end
    local ok, err = pcall(setRestriction, false)
    if not ok then print("[eddie_spawner] couldn't give V's movement back: " .. tostring(err)) end
    local r, player = active(), Game.GetPlayer()
    if r and player and Game.FindEntityByID(r.id) then
        local x, y, z = stepOffSpot(r)
        Game.GetTeleportationFacility():Teleport(player, Vector4.new(x, y, z, 1), EulerAngles.new(0, 0, ride.yaw))
    end
    say(msg or "Got off Eddie")
end

local function settingValue(group, name, default)
    local ok, v = pcall(function() return Game.GetSettingsSystem():GetVar(group, name):GetValue() end)
    return (ok and type(v) == "number" and v > 0) and v or default
end

-- player input (PlayerPuppet:OnAction), only used while riding
local BUTTONS = { Forward = "fwd", Back = "back", Left = "left", Right = "right" }
local function onAction(name, kind, value)
    if not riding then return end
    local pressed = kind == "BUTTON_PRESSED"
    local moved = false
    if BUTTONS[name] then
        if pressed then ride[BUTTONS[name]] = 1 elseif kind == "BUTTON_RELEASED" then ride[BUTTONS[name]] = 0 end
        moved = pressed
    elseif name == "MoveY" then -- left stick
        ride.fwd, ride.back = math.max(0, value), math.max(0, -value)
        moved = math.abs(value) > 0.3
    elseif name == "MoveX" then
        ride.right, ride.left = math.max(0, value), math.max(0, -value)
        moved = math.abs(value) > 0.3
    elseif name == "Jump" and pressed then
        ride.getOff = true
    elseif name == "CameraMouseX" then -- mouse delta; V's own turning is off while she's moved every frame
        ride.mouse = ride.mouse - value / 35 * ride.mouseSens
    elseif name == "right_stick_x" then
        ride.stick = -value * 1.7 * ride.padSens
    end
    if moved and mode ~= "ride" then mode, goal = "ride", nil end -- take over from goto / manual
end

-- V's view yaw this frame: where she looked last frame plus the mouse / stick since then
local function updateRideYaw(dt)
    ride.yaw = wrap(Game.GetPlayer():GetWorldYaw() + ride.mouse + ride.stick * dt * 60)
    ride.mouse = 0
end

-- put V on her spot on Eddie, looking where she looks
local function placeRider(r)
    local fx, fy = dirFromYaw(r.yaw)
    local rx, ry = fy, -fx
    local s = RIDE_SPOT
    local x, y = r.pos.x + rx * s[1] + fx * s[2], r.pos.y + ry * s[1] + fy * s[2]
    Game.GetTeleportationFacility():Teleport(Game.GetPlayer(), Vector4.new(x, y, r.pos.z + s[3], 1),
        EulerAngles.new(0, 0, ride.yaw))
end

local function spawn()
    local player = Game.GetPlayer()
    if not player then return say("No player yet: load a save first.") end
    stopRiding()
    calibrate(player)

    local pos = player:GetWorldPosition()
    local fwd = player:GetWorldForward()
    local x, y = pos.x + fwd.x * SPAWN_DISTANCE, pos.y + fwd.y * SPAWN_DISTANCE
    local yaw = player:GetWorldYaw() + 180 -- face V

    local transform = player:GetWorldTransform()
    transform:SetPosition(Vector4.new(x, y, pos.z, 1))
    transform:SetOrientationEuler(EulerAngles.new(0, 0, yaw))

    local ok, id = pcall(exEntitySpawner.Spawn, ENT_PATH, transform, "default")
    if not (ok and id) then return say("Spawn failed: " .. tostring(id)) end
    table.insert(robots, { id = id, pos = { x = x, y = y, z = pos.z }, yaw = yaw, vel = { x = 0, y = 0 } })
    say(string.format("Spawned Eddie #%d (drives the newest one)", #robots))
end

local function despawnAll()
    stopRiding()
    local n = 0
    for _, r in ipairs(robots) do
        local e = Game.FindEntityByID(r.id)
        if e then exEntitySpawner.Despawn(e); n = n + 1 end
    end
    robots, mode, goal = {}, "idle", nil
    say(string.format("Despawned %d", n))
end

local function setMode(m)
    if riding and m == "follow" then return say("Get off Eddie first to have it follow you.") end
    mode = m
    if m ~= "goto" then goal = nil end
    say("Mode: " .. m)
end

local function sendToLook()
    local player = Game.GetPlayer()
    if not player or not active() then return say("Spawn Eddie first.") end
    local p = player:GetWorldPosition()
    local eye = Vector4.new(p.x, p.y, p.z + 1.6, 1)
    local d = Game.GetCameraSystem():GetActiveCameraForward()
    local hit = raycast(eye, Vector4.new(eye.x + d.x * 40, eye.y + d.y * 40, eye.z + d.z * 40, 1))
    if not hit then return say("Nothing under the crosshair within 40 m.") end
    goal = { x = hit.x, y = hit.y }
    mode = "goto"
    say(string.format("Driving to %.1f m away", math.sqrt((hit.x - active().pos.x) ^ 2 + (hit.y - active().pos.y) ^ 2)))
end

local function startRiding()
    local player, r = Game.GetPlayer(), active()
    if not player or not r then return say("Spawn Eddie first.") end
    if not Game.FindEntityByID(r.id) then return say("Eddie isn't loaded here.") end
    local p = player:GetWorldPosition()
    if math.sqrt((p.x - r.pos.x) ^ 2 + (p.y - r.pos.y) ^ 2) > RIDE_REACH then
        return say(string.format("Walk up to Eddie first (within %d m).", RIDE_REACH))
    end
    -- make sure the movement buttons reach PlayerPuppet:OnAction
    for _, a in ipairs(RIDE_ACTIONS) do
        pcall(function()
            player:UnregisterInputListener(player, a)
            player:RegisterInputListener(player, a)
        end)
    end
    ride.fwd, ride.back, ride.left, ride.right = 0, 0, 0, 0
    ride.mouse, ride.stick, ride.getOff = 0, 0, false
    ride.yaw = player:GetWorldYaw()
    ride.mouseSens = settingValue("/controls/fppcameramouse", "FPP_MouseX", 2.9) / 2.9
    ride.padSens = settingValue("/controls/fppcamerapad", "FPP_PadX", 10) / 10
    local ok, err = pcall(setRestriction, true)
    riding = true
    mode, goal = "ride", nil
    say("Riding: W/A/S/D drive, the mouse steers, Jump gets off")
    if not ok then say("Riding, but V's walking couldn't be switched off: " .. tostring(err)) end
end

local function toggleRide()
    if riding then return stopRiding() end
    if active() then return startRiding() end
    -- nothing to ride: make sure V isn't left without her movement (e.g. a save made while riding)
    pcall(setRestriction, false)
    say("Spawn Eddie first.")
end

-- ---------------------------------------------------------------------------
-- Drive units and wheels
-- Layout in the URDF base frame (x forward, y left, z up); the entity's mesh space is
-- that turned +90 deg about z (front = +Y). Every unit uses the unmirrored drive mesh;
-- the mirrored ones start at steer = pi.
-- ---------------------------------------------------------------------------
local partNames = nil -- { [unit] = { body = CName, r = CName, l = CName } }

local function wrapRad(a)
    while a > math.pi do a = a - 2 * math.pi end
    while a < -math.pi do a = a + 2 * math.pi end
    return a
end

local function cname(s)
    if CName and CName.new then return CName.new(s) end
    return s
end

local function setPart(entity, name, x, y, z, yaw, spin)
    local comp = entity:FindComponentByName(name)
    if not comp then return false end
    -- Rz(yaw) * Rx(spin), as a quaternion (i, j, k, r)
    local cz, sz = math.cos(yaw / 2), math.sin(yaw / 2)
    local cx, sx = math.cos(spin / 2), math.sin(spin / 2)
    comp:SetLocalTransform(Vector4.new(-y, x, z, 1), Quaternion.new(cz * sx, sz * sx, sz * cx, cz * cx))
    return true
end

-- vx, vy: body velocity in the base frame (m/s); w: yaw rate, counter-clockwise (rad/s)
local function animateParts(r, entity, vx, vy, w, dt)
    if not PARTS or r.noParts or dt <= 0 then return end
    if not partNames then
        partNames = {}
        for i, d in ipairs(PARTS.drives) do
            local base = PARTS.prefix .. d.key
            partNames[i] = { body = cname(base), r = cname(base .. "_r"), l = cname(base .. "_l") }
        end
    end
    if not r.units then
        r.units = {}
        for i, d in ipairs(PARTS.drives) do r.units[i] = { steer = d.steer0, spin = { r = 0, l = 0 } } end
    end
    if math.abs(vx) + math.abs(vy) < 1e-4 and math.abs(w) < 1e-4 then return end -- parked: castors stay put
    local castor = math.abs(PARTS.wheel_offsets.r[2])
    for i, d in ipairs(PARTS.drives) do
        local u, px, py, pz = r.units[i], d.pivot[1], d.pivot[2], d.pivot[3]
        -- the pivot's ground velocity; the wheels trail it like a castor
        local vpx, vpy = vx - w * py, vy + w * px
        local sp = math.sqrt(vpx * vpx + vpy * vpy)
        local dSteer = 0
        if sp > 1e-3 then
            -- steer that puts the wheel offset (0, +castor in the unit frame) behind the pivot
            local err = wrapRad(atan2(vpx, -vpy) - u.steer)
            dSteer = err * math.min(1, sp / castor * dt)
            local cap = STEER_RATE_MAX * dt
            dSteer = math.max(-cap, math.min(cap, dSteer))
            u.steer = wrapRad(u.steer + dSteer)
        end
        local c, s = math.cos(u.steer), math.sin(u.steer)
        local yaw = math.pi / 2 + u.steer
        if not setPart(entity, partNames[i].body, px, py, pz, yaw, 0) then
            r.noParts = true
            return say("No drive/wheel components in eddie.ent: rerun install_wolvenkit.py")
        end
        local wr = w + dSteer / dt -- the unit's own turn rate
        for _, side in ipairs({ "r", "l" }) do
            local o = PARTS.wheel_offsets[side]
            local ox, oy = c * o[1] - s * o[2], s * o[1] + c * o[2]
            local wvx, wvy = vpx - wr * oy, vpy + wr * ox
            local roll = -s * wvx + c * wvy -- along the unit's rolling direction
            -- spin axis is the unit's x: rolling forward is a negative turn about it
            u.spin[side] = (u.spin[side] - roll / PARTS.wheel_radius * dt) % (2 * math.pi)
            setPart(entity, partNames[i][side], px + ox, py + oy, pz + o[3], yaw, u.spin[side])
        end
    end
end

-- ---------------------------------------------------------------------------
-- Arms: a chain of parts (eddie_parts.lua -> arms.links), each turned about its joint
-- axis by its joint variable; poses (solved offline by build_glb.py) set the variables.
-- Both arms move together so they arrive at the same time.
-- ---------------------------------------------------------------------------
local ARMS = PARTS and PARTS.arms
local ARM_SPEED = 1.0  -- rad/s, for the joint with the furthest to go
local GRIP_SPEED = 1.5 -- rad/s
local POSE_LABELS = { { "rest", "Rest" }, { "reach", "Reach" }, { "up", "Arms up" }, { "wave", "Wave" } }
local armNames = nil
local SQRT_HALF = math.sqrt(0.5)
if ARMS then -- unit quaternions, whatever the rounding in the file
    for _, L in ipairs(ARMS.links) do
        local o = L.origin
        local n = math.sqrt(o[4] ^ 2 + o[5] ^ 2 + o[6] ^ 2 + o[7] ^ 2)
        for k = 4, 7 do o[k] = o[k] / n end
    end
end

local function qmul(a, b)
    return { a[4] * b[1] + a[1] * b[4] + a[2] * b[3] - a[3] * b[2],
             a[4] * b[2] - a[1] * b[3] + a[2] * b[4] + a[3] * b[1],
             a[4] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[4],
             a[4] * b[4] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3] }
end

local function qrot(q, v)
    local x, y, z = q[1], q[2], q[3]
    local tx, ty, tz = 2 * (y * v[3] - z * v[2]), 2 * (z * v[1] - x * v[3]), 2 * (x * v[2] - y * v[1])
    return { v[1] + q[4] * tx + y * tz - z * ty, v[2] + q[4] * ty + z * tx - x * tz,
             v[3] + q[4] * tz + x * ty - y * tx }
end

-- every part's transform in Eddie's base frame (x forward, y left, z up) for joint values q
local function armFK(q)
    local out = {}
    for i, L in ipairs(ARMS.links) do
        local o = L.origin
        local p, rot = { o[1], o[2], o[3] }, { o[4], o[5], o[6], o[7] }
        if L.var then
            local half = L.scale * q[L.var] / 2
            local s = math.sin(half)
            rot = qmul(rot, { L.axis[1] * s, L.axis[2] * s, L.axis[3] * s, math.cos(half) })
        end
        if L.parent > 0 then
            local P = out[L.parent]
            local d = qrot(P.q, p)
            p, rot = { P.p[1] + d[1], P.p[2] + d[2], P.p[3] + d[3] }, qmul(P.q, rot)
        end
        out[i] = { p = p, q = rot }
    end
    return out
end

local function armState(r)
    if not r.arm then
        local q = {}
        for k, v in pairs(ARMS.vars) do q[k] = v end
        r.arm = { q = q, target = "rest", grip = 0, wave = 0, t = 0 }
    end
    return r.arm
end

local function setArms(name)
    local r = active()
    if not r then return say("Spawn Eddie first.") end
    if not ARMS then return say("No arm parts: rerun build_glb.py and install_wolvenkit.py") end
    armState(r).target = name
    say("Arms: " .. name)
end

local function toggleGrip()
    local r = active()
    if not r then return say("Spawn Eddie first.") end
    if not ARMS then return say("No arm parts: rerun build_glb.py and install_wolvenkit.py") end
    local a = armState(r)
    a.grip = a.grip > 0 and 0 or ARMS.grip_closed
    say(a.grip > 0 and "Grippers: closing" or "Grippers: opening")
end

local function moveArms(r, entity, dt)
    if not ARMS or r.noArms or dt <= 0 then return end
    local a = armState(r)
    local moved = false
    -- joints: all move together, at ARM_SPEED for the one with the furthest to go
    local pose, far = ARMS.poses[a.target], 0
    for k, v in pairs(pose) do far = math.max(far, math.abs(v - a.q[k])) end
    if far > 1e-5 then
        local f = math.min(1, ARM_SPEED * dt / far)
        for k, v in pairs(pose) do a.q[k] = a.q[k] + (v - a.q[k]) * f end
        moved = true
    end
    for _, g in ipairs({ "lg", "rg" }) do
        local d = a.grip - a.q[g]
        if math.abs(d) > 1e-5 then
            a.q[g] = a.q[g] + math.max(-GRIP_SPEED * dt, math.min(GRIP_SPEED * dt, d))
            moved = true
        end
    end
    -- waving: swing one joint once the arm is up; ease back to 0 when it stops
    local w = ARMS.wave
    local goal = 0
    if a.target == "wave" and far <= 1e-5 then
        a.t = a.t + dt
        goal = w.amplitude * math.sin(2 * math.pi * w.hz * a.t)
    else
        a.t = 0
    end
    if math.abs(goal - a.wave) > 1e-5 then
        local cap = (a.t > 0 and 2 * math.pi * w.hz * w.amplitude * 1.2 or ARM_SPEED) * dt
        a.wave = a.wave + math.max(-cap, math.min(cap, goal - a.wave))
        moved = true
    end
    if not moved then return end

    local q = a.q
    if a.wave ~= 0 then
        q = {}
        for k, v in pairs(a.q) do q[k] = v end
        q[w.var] = q[w.var] + a.wave
    end
    if not armNames then
        armNames = {}
        for i, L in ipairs(ARMS.links) do armNames[i] = cname(PARTS.prefix .. L.name) end
    end
    for i, T in ipairs(armFK(q)) do
        local comp = entity:FindComponentByName(armNames[i])
        if not comp then
            r.noArms = true
            return say("No arm components in eddie.ent: rerun install_wolvenkit.py")
        end
        -- entity space = Eddie's base frame turned +90 deg about z
        local x, y, z, k = T.q[1], T.q[2], T.q[3], T.q[4]
        comp:SetLocalTransform(Vector4.new(-T.p[2], T.p[1], T.p[3], 1),
            Quaternion.new(SQRT_HALF * (x - y), SQRT_HALF * (x + y), SQRT_HALF * (z + k), SQRT_HALF * (k - z)))
    end
end

-- desired world velocity and heading for this frame
local function command(r, dt)
    local vx, vy, faceYaw, turn = 0, 0, nil, 0
    if mode == "follow" then
        local p = Game.GetPlayer():GetWorldPosition()
        local dx, dy = p.x - r.pos.x, p.y - r.pos.y
        local dist = math.sqrt(dx * dx + dy * dy)
        local s = speed * math.max(0, math.min(1, (dist - FOLLOW_DISTANCE) / 1.0))
        if dist > 1e-3 then
            vx, vy = dx / dist * s, dy / dist * s
            faceYaw = yawFromDir(dx, dy) -- keeps looking at V
        end
    elseif mode == "goto" and goal then
        local dx, dy = goal.x - r.pos.x, goal.y - r.pos.y
        local dist = math.sqrt(dx * dx + dy * dy)
        if dist < 0.1 then
            setMode("idle")
            say("Arrived")
        else
            local s = speed * math.min(1, dist / 0.8)
            vx, vy = dx / dist * s, dy / dist * s
            faceYaw = yawFromDir(dx, dy)
        end
    elseif mode == "manual" then
        local f = (keys.fwd and 1 or 0) - (keys.back and 1 or 0)
        local st = (keys.right and 1 or 0) - (keys.left and 1 or 0)
        local fx, fy = dirFromYaw(r.yaw)
        local rx, ry = fy, -fx -- right-hand side (Z up)
        vx, vy = (fx * f + rx * st) * speed, (fy * f + ry * st) * speed
        turn = ((keys.tleft and 1 or 0) - (keys.tright and 1 or 0)) * TURN_RATE * -yawSign
    elseif mode == "ride" then
        -- relative to where V looks; Eddie turns to face the same way
        local f, st = ride.fwd - ride.back, ride.right - ride.left
        local n = math.sqrt(f * f + st * st)
        if n > 1 then f, st = f / n, st / n end
        local fx, fy = dirFromYaw(ride.yaw)
        local rx, ry = fy, -fx
        vx, vy = (fx * f + rx * st) * speed, (fy * f + ry * st) * speed
        faceYaw = ride.yaw
    end
    return vx, vy, faceYaw, turn
end

local function drive(dt)
    local r = active()
    if not r or not riding and mode == "idle" and r.vel.x == 0 and r.vel.y == 0 then return end
    local entity = Game.FindEntityByID(r.id)
    if not entity then
        if riding then stopRiding("Eddie is gone: you got off") end
        return
    end
    if riding then updateRideYaw(dt) end

    local vx, vy, faceYaw, turn = command(r, dt)
    local x0, y0, yaw0 = r.pos.x, r.pos.y, r.yaw

    -- accelerate towards the commanded velocity
    local ax, ay = vx - r.vel.x, vy - r.vel.y
    local a = math.sqrt(ax * ax + ay * ay)
    local maxDv = ACCEL * dt
    if a > maxDv then ax, ay = ax / a * maxDv, ay / a * maxDv end
    r.vel.x, r.vel.y = r.vel.x + ax, r.vel.y + ay
    if math.abs(r.vel.x) < 1e-3 and math.abs(r.vel.y) < 1e-3 and vx == 0 and vy == 0 then
        r.vel.x, r.vel.y = 0, 0
    end

    -- heading
    if faceYaw then
        local d = wrap(faceYaw - r.yaw)
        local step = TURN_RATE * dt
        r.yaw = r.yaw + math.max(-step, math.min(step, d))
    end
    r.yaw = wrap(r.yaw + turn * dt)

    -- move unless a wall or a drop is in the way
    local nx, ny = r.pos.x + r.vel.x * dt, r.pos.y + r.vel.y * dt
    if r.vel.x ~= 0 or r.vel.y ~= 0 then
        local gz = groundAt(nx, ny, r.pos.z)
        if gz and not blocked(r, r.vel.x, r.vel.y) then
            r.pos.x, r.pos.y, r.pos.z = nx, ny, gz
        else
            r.vel.x, r.vel.y = 0, 0
            if mode == "goto" then setMode("idle"); say("Blocked, stopped") end
        end
    end

    Game.GetTeleportationFacility():Teleport(entity, Vector4.new(r.pos.x, r.pos.y, r.pos.z, 1),
        EulerAngles.new(0, 0, r.yaw))

    -- what actually happened this frame, in the robot's own frame, drives the wheels
    local dx, dy = r.pos.x - x0, r.pos.y - y0
    local fx, fy = dirFromYaw(r.yaw)
    local bvx = (dx * fx + dy * fy) / dt
    local bvy = (dx * -fy + dy * fx) / dt -- left = forward turned 90 deg counter-clockwise
    local bw = -yawSign * math.rad(wrap(r.yaw - yaw0)) / dt
    animateParts(r, entity, bvx, bvy, bw, dt)
    if riding then placeRider(r) end
end

registerForEvent("onInit", function()
    local ok, err = pcall(Observe, "PlayerPuppet", "OnAction", function(_, action)
        if not riding then return end
        onAction(Game.NameToString(ListenerAction.GetName(action)), ListenerAction.GetType(action).value,
            ListenerAction.GetValue(action))
    end)
    print("[eddie_spawner] loaded" .. (ok and "" or " (no ride controls: " .. tostring(err) .. ")"))
end)
registerForEvent("onShutdown", function()
    if riding then pcall(setRestriction, false) end
end)
registerForEvent("onOverlayOpen", function() overlayOpen = true end)
registerForEvent("onOverlayClose", function() overlayOpen = false end)

registerForEvent("onUpdate", function(dt)
    if #robots == 0 or not Game.GetPlayer() then return end
    local srh = Game.GetSystemRequestsHandler()
    if srh:IsPreGame() or srh:IsGamePaused() then return end
    if ride.getOff then stopRiding() end
    local ok, err = pcall(drive, math.min(dt, 0.1))
    if not ok then
        pcall(stopRiding)
        setMode("idle")
        say("Driving error: " .. tostring(err))
    end
    local r = active()
    local entity = r and Game.FindEntityByID(r.id)
    if entity then
        ok, err = pcall(moveArms, r, entity, math.min(dt, 0.1))
        if not ok then
            r.noArms = true
            say("Arm error: " .. tostring(err))
        end
    end
end)

registerForEvent("onDraw", function()
    if not overlayOpen then return end
    if ImGui.Begin("Eddie") then
        if ImGui.Button("Spawn in front of V") then spawn() end
        ImGui.SameLine()
        if ImGui.Button("Despawn all") then despawnAll() end
        ImGui.Text(string.format("Active: %d   Mode: %s", #robots, mode))
        ImGui.Separator()
        if ImGui.Button("Follow V") then setMode("follow") end
        ImGui.SameLine()
        if ImGui.Button("Go where I look") then sendToLook() end
        ImGui.SameLine()
        if ImGui.Button("Manual") then setMode("manual") end
        ImGui.SameLine()
        if ImGui.Button("Stop") then setMode("idle") end
        if ImGui.Button(riding and "Get off" or "Ride") then toggleRide() end
        speed = ImGui.SliderFloat("Speed (m/s)", speed, 0.2, 2.0)
        ImGui.TextWrapped("Manual: hold the keys bound under Bindings -> eddie_spawner.")
        ImGui.TextWrapped("Ride: W/A/S/D drive, the mouse steers, Jump gets off.")
        if ARMS then
            ImGui.Separator()
            ImGui.Text("Arms")
            for i, p in ipairs(POSE_LABELS) do
                if i > 1 then ImGui.SameLine() end
                if ImGui.Button(p[2]) then setArms(p[1]) end
            end
            ImGui.SameLine()
            local r = active()
            local closed = r and r.arm and r.arm.grip > 0
            if ImGui.Button(closed and "Open grippers" or "Close grippers") then toggleGrip() end
        end
        if status ~= "" then ImGui.TextWrapped(status) end
    end
    ImGui.End()
end)

registerHotkey("spawn_eddie", "Spawn Eddie in front of V", spawn)
registerHotkey("despawn_eddie", "Despawn all Eddies", despawnAll)
registerHotkey("eddie_follow", "Follow V on/off", function() setMode(mode == "follow" and "idle" or "follow") end)
registerHotkey("eddie_goto", "Drive to where V looks", sendToLook)
registerHotkey("eddie_stop", "Stop", function() setMode("idle") end)
registerHotkey("eddie_ride", "Ride Eddie / get off", toggleRide)
for _, p in ipairs(POSE_LABELS) do
    registerHotkey("eddie_arms_" .. p[1], "Arms: " .. p[2], function() setArms(p[1]) end)
end
registerHotkey("eddie_grip", "Grippers open / close", toggleGrip)

local function holdKey(name)
    return function(down)
        keys[name] = down
        if down and mode ~= "manual" then setMode("manual") end
    end
end
registerInput("eddie_fwd", "Drive forward (hold)", holdKey("fwd"))
registerInput("eddie_back", "Drive back (hold)", holdKey("back"))
registerInput("eddie_left", "Strafe left (hold)", holdKey("left"))
registerInput("eddie_right", "Strafe right (hold)", holdKey("right"))
registerInput("eddie_turn_left", "Turn left (hold)", holdKey("tleft"))
registerInput("eddie_turn_right", "Turn right (hold)", holdKey("tright"))
