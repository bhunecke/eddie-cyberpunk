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
--
-- Weapons: "Give" hands Eddie a copy of the weapon V has drawn, held in a gripper (a loot
-- drop teleported to the fingers every frame); "Drop weapons" lets go of it.

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
local mode = "idle"         -- idle | follow | guard | goto | manual | ride
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

local dropWeapons, fallingCount -- defined with the weapons below

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
    for _, r in ipairs(robots) do -- let go of any weapons first, so they fall to the floor
        if r.weapons and next(r.weapons) then dropWeapons(r) end
    end
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
    if riding and m == "guard" then return say("Get off Eddie first to put it on guard.") end
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
local HOLD_CLOSURE = 0.45 -- gripper closure on a held weapon (0 open .. 0.8 closed)
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

local function holding(r)
    return r.weapons ~= nil and next(r.weapons) ~= nil
end

local function toggleGrip()
    local r = active()
    if not r then return say("Spawn Eddie first.") end
    if not ARMS then return say("No arm parts: rerun build_glb.py and install_wolvenkit.py") end
    local a = armState(r)
    if holding(r) then -- opening the grippers lets go of the weapons
        a.grip = 0
        return dropWeapons(r, "Grippers: opening, weapons dropped")
    end
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
        local d = (r.weapons and r.weapons[g:sub(1, 1)] and HOLD_CLOSURE or a.grip) - a.q[g]
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
    a.fk = armFK(q)
    if not armNames then
        armNames = {}
        for i, L in ipairs(ARMS.links) do armNames[i] = cname(PARTS.prefix .. L.name) end
    end
    for i, T in ipairs(a.fk) do
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

-- ---------------------------------------------------------------------------
-- Weapons: Eddie gets a copy of the weapon V has drawn (a fresh item of the same TweakDB
-- record, so V keeps hers). It's spawned as a loot drop and teleported every frame to
-- the gripper's tool point, the barrel along the fingers. "Drop weapons" (or opening
-- the grippers) lets go: the drop falls and V can pick it up.
-- ---------------------------------------------------------------------------
local WEAPON_SLOT = "AttachmentSlots.WeaponRight"
local WEAPON_OFFSET = { 0.0, 0.0, 0.0 } -- m, weapon origin from the tool point, in the weapon's frame (right, forward, up)
local WEAPON_SPAWN_WAIT = 3.0           -- s to wait for the drop to show up
local HAND_NAMES = { r = "right", l = "left" }
local GRIP_LINK, WEAPON_IN_GRIP = {}, {}

-- unit quaternion (i, j, k, r) from the rotated frame's axes (the matrix columns)
local function quatFromAxes(X, Y, Z)
    local tr = X[1] + Y[2] + Z[3]
    local q
    if tr > 0 then
        local s = math.sqrt(tr + 1) * 2
        q = { (Y[3] - Z[2]) / s, (Z[1] - X[3]) / s, (X[2] - Y[1]) / s, s / 4 }
    elseif X[1] > Y[2] and X[1] > Z[3] then
        local s = math.sqrt(1 + X[1] - Y[2] - Z[3]) * 2
        q = { s / 4, (Y[1] + X[2]) / s, (Z[1] + X[3]) / s, (Y[3] - Z[2]) / s }
    elseif Y[2] > Z[3] then
        local s = math.sqrt(1 + Y[2] - X[1] - Z[3]) * 2
        q = { (Y[1] + X[2]) / s, s / 4, (Z[2] + Y[3]) / s, (Z[1] - X[3]) / s }
    else
        local s = math.sqrt(1 + Z[3] - X[1] - Y[2]) * 2
        q = { (Z[1] + X[3]) / s, (Z[2] + Y[3]) / s, s / 4, (X[2] - Y[1]) / s }
    end
    return q
end

if ARMS then
    for i, L in ipairs(ARMS.links) do
        if L.name == "l_grip_base" then GRIP_LINK.l = i elseif L.name == "r_grip_base" then GRIP_LINK.r = i end
    end
    -- The weapon's forward (+Y) runs along the fingers (gripper +z), turned about that axis
    -- so that it's upright in the Reach pose (the grippers' opening direction is only
    -- roughly vertical there): its up (+Z) is world up seen from the gripper, minus the
    -- part along the fingers.
    local q = {}
    for k, v in pairs(ARMS.vars) do q[k] = v end
    for k, v in pairs(ARMS.poses.reach or {}) do q[k] = v end
    local fk = armFK(q)
    for side, i in pairs(GRIP_LINK) do
        local g = fk[i].q
        local u = qrot({ -g[1], -g[2], -g[3], g[4] }, { 0, 0, 1 })
        local n = math.sqrt(u[1] ^ 2 + u[2] ^ 2)
        if n < 1e-6 then u, n = { 1, 0, 0 }, 1 end -- fingers straight up: any roll will do
        u = { u[1] / n, u[2] / n, 0 }
        WEAPON_IN_GRIP[side] = quatFromAxes({ -u[2], u[1], 0 }, { 0, 0, 1 }, u)
    end
end

-- the game's own conversion, in whichever form this CET exposes it
local function toEuler(q)
    local Q = Quaternion.new(q[1], q[2], q[3], q[4])
    for _, f in ipairs({
        function() return Q:ToEulerAngles() end,
        function() return Quaternion.ToEulerAngles(Q) end,
        function() return GetSingleton("Quaternion"):ToEulerAngles(Q) end,
    }) do
        local ok, e = pcall(f)
        if ok and e then return e end
    end
    error("no Quaternion -> EulerAngles conversion")
end

-- SpawnItemDrop only returns whether it worked; the drop object (gameItemDropObject)
-- appears a few frames later. Its OnGameAttached claims it for the hand that asked for a
-- drop at that spot.
local CLAIM_RADIUS = 0.75 -- m between where we asked for the drop and where it appeared
local pendingDrops = {}   -- { w = hand entry, pos = {x, y, z} }

local function claimDrop(obj)
    if #pendingDrops == 0 then return end
    local p = obj:GetWorldPosition()
    for i, d in ipairs(pendingDrops) do
        if (p.x - d.pos[1]) ^ 2 + (p.y - d.pos[2]) ^ 2 + (p.z - d.pos[3]) ^ 2 < CLAIM_RADIUS ^ 2 then
            d.w.id = obj:GetEntityID()
            local okK, key = pcall(entityKey, obj)
            if okK then d.w.key = key end
            table.remove(pendingDrops, i)
            print("[eddie_spawner] caught the weapon drop")
            return
        end
    end
end

-- world position and orientation of a weapon held in the given hand
local function weaponPose(r, side)
    local a = armState(r)
    if not a.fk then a.fk = armFK(a.q) end
    local T, qw = a.fk[GRIP_LINK[side]], WEAPON_IN_GRIP[side]
    local o = qrot(qw, WEAPON_OFFSET)
    local t = ARMS.tool
    local d = qrot(T.q, { t[1] + o[1], t[2] + o[2], t[3] + o[3] })
    local pb = { T.p[1] + d[1], T.p[2] + d[2], T.p[3] + d[3] } -- base frame
    -- base frame -> world: the entity's heading (counter-clockwise) plus the +90 deg to its mesh space
    local phi = -yawSign * math.rad(r.yaw) + math.pi / 2
    local qe = { 0, 0, math.sin(phi / 2), math.cos(phi / 2) }
    local pw = qrot(qe, pb)
    return { r.pos.x + pw[1], r.pos.y + pw[2], r.pos.z + pw[3] }, qmul(qe, qmul(T.q, qw))
end

-- the item V holds in her right hand, as a fresh copy (new seed, same record)
local function copyOfVsWeapon(player)
    local obj = Game.GetTransactionSystem():GetItemInSlot(player, TweakDBID.new(WEAPON_SLOT))
    if not obj then return nil end
    local id = obj:GetItemID()
    local ok, tdb = pcall(ItemID.GetTDBID, id)
    if not ok or not tdb then tdb = id.id end
    return ItemID.FromTDBID(tdb)
end

local function isA(obj, cls)
    local ok, yes = pcall(function() return obj:IsA(cls) end)
    return ok and yes == true
end

-- Fallback when OnGameAttached didn't catch it: look for an item drop near where it was
-- spawned with a targeting query around V (the loose weapon is an ItemObject connected
-- to its drop)
local SCAN_EVERY, SCAN_RADIUS = 0.25, 2.0 -- s, m from the spawn point

local function entityKey(e)
    local id = e:GetEntityID()
    local ok, h = pcall(function() return id.hash end)
    return tostring(ok and h or id)
end

-- entities the targeting system knows around V, within maxDistance (m)
local function entitiesAroundV(maxDistance)
    local player = Game.GetPlayer()
    local query = gameTargetSearchQuery.new()
    query.maxDistance = maxDistance
    query.filterObjectByDistance = true
    query.includeSecondaryTargets = false
    query.ignoreInstigator = true
    pcall(function() query.testedSet = gameTargetingSet.Complete end)
    local success, parts = Game.GetTargetingSystem():GetTargetParts(player, query)
    local out, seen = {}, {}
    if not success or not parts then return out end
    for _, part in ipairs(parts) do
        local comp = gametargetingTargetPartInfo.GetComponent(part)
        local ent = comp and comp:GetEntity()
        if ent then
            local ok, key = pcall(entityKey, ent)
            if not ok or not seen[key] then
                if ok then seen[key] = true end
                out[#out + 1] = ent
            end
        end
    end
    return out
end

-- item drops (gameItemDropObject) within SCAN_RADIUS of `at` (and up to 2.5 m below it):
-- { { drop, key, d2 } }
local function dropsNear(at)
    local out = {}
    for _, ent in ipairs(entitiesAroundV(30)) do
        local drop = nil
        if ent and isA(ent, "gameItemDropObject") then
            drop = ent
        elseif ent and isA(ent, "gameItemObject") then
            local ok, d = pcall(function() return ent:GetConnectedItemDrop() end)
            if ok then drop = d end
        end
        if drop then
            local p = drop:GetWorldPosition()
            local d = (p.x - at[1]) ^ 2 + (p.y - at[2]) ^ 2
            if d < SCAN_RADIUS ^ 2 and p.z > at[3] - 2.5 and p.z < at[3] + 0.5 then
                out[#out + 1] = { drop = drop, key = entityKey(drop), d2 = d }
            end
        end
    end
    return out
end

-- the nearest drop that wasn't there before we asked for this one, and isn't held already
local function scanForDrop(w, at, taken)
    local best = nil
    for _, c in ipairs(dropsNear(at)) do
        if not w.before[c.key] and not taken[c.key] and (not best or c.d2 < best.d2) then best = c end
    end
    if best then
        w.id, w.key = best.drop:GetEntityID(), best.key
        print("[eddie_spawner] found the weapon drop by scanning")
    end
end

local function forgetPending(w)
    for i = #pendingDrops, 1, -1 do
        if pendingDrops[i].w == w then table.remove(pendingDrops, i) end
    end
end

local function dropWeapon(r, side)
    if r.weapons and r.weapons[side] then
        forgetPending(r.weapons[side])
        r.weapons[side] = nil
    end
end

-- Once it has been moved every frame the game doesn't let the drop fall again, so a
-- released weapon is dropped here: it falls under gravity onto the floor below the
-- gripper, turning onto its side on the way, and stays there to be picked up.
local GRAVITY, FLOOR_LIFT = 9.81, 0.03 -- m/s^2, m above the floor it comes to rest at
local falling = {} -- { id, x, y, z, z0, z1, v, q0, q1 }

local function lyingQuat(q)
    local f = qrot(q, { 0, 1, 0 }) -- barrel
    local n = math.sqrt(f[1] ^ 2 + f[2] ^ 2)
    local Y = n > 1e-6 and { f[1] / n, f[2] / n, 0 } or { 0, 1, 0 }
    -- barrel level, on its side: right side up
    return quatFromAxes({ 0, 0, 1 }, Y, { -Y[2], Y[1], 0 })
end

local function slerp(a, b, t)
    local d = a[1] * b[1] + a[2] * b[2] + a[3] * b[3] + a[4] * b[4]
    if d < 0 then b, d = { -b[1], -b[2], -b[3], -b[4] }, -d end
    local wa, wb
    if d > 0.9995 then
        wa, wb = 1 - t, t
    else
        local th = math.acos(d)
        wa, wb = math.sin((1 - t) * th) / math.sin(th), math.sin(t * th) / math.sin(th)
    end
    local q = { wa * a[1] + wb * b[1], wa * a[2] + wb * b[2], wa * a[3] + wb * b[3], wa * a[4] + wb * b[4] }
    local n = math.sqrt(q[1] ^ 2 + q[2] ^ 2 + q[3] ^ 2 + q[4] ^ 2)
    return { q[1] / n, q[2] / n, q[3] / n, q[4] / n }
end

local function releaseWeapon(r, side)
    local w = r.weapons and r.weapons[side]
    if w and w.id and w.seen then
        local p, q = weaponPose(r, side)
        local ok, hit = pcall(raycast, Vector4.new(p[1], p[2], p[3], 1), Vector4.new(p[1], p[2], p[3] - 4, 1))
        local floor = (ok and hit) and hit.z or r.pos.z
        local z1 = math.min(p[3], floor + FLOOR_LIFT)
        table.insert(falling, { id = w.id, x = p[1], y = p[2], z = p[3], z0 = p[3], z1 = z1, v = 0,
                                q0 = q, q1 = lyingQuat(q) })
    end
    dropWeapon(r, side)
end

fallingCount = function() return #falling end

local function updateFalling(dt)
    for i = #falling, 1, -1 do
        local f = falling[i]
        local e = Game.FindEntityByID(f.id)
        if not e then
            table.remove(falling, i) -- picked up
        else
            f.v = f.v + GRAVITY * dt
            f.z = math.max(f.z1, f.z - f.v * dt)
            local t = f.z0 > f.z1 and (f.z0 - f.z) / (f.z0 - f.z1) or 1
            Game.GetTeleportationFacility():Teleport(e, Vector4.new(f.x, f.y, f.z, 1), toEuler(slerp(f.q0, f.q1, t)))
            if f.z <= f.z1 then table.remove(falling, i) end
        end
    end
end

dropWeapons = function(r, msg)
    if not r or not holding(r) then return say(msg or "Eddie isn't holding a weapon.") end
    for side in pairs(r.weapons) do releaseWeapon(r, side) end
    r.weapons = {}
    say(msg or "Weapons dropped")
end

local function giveWeapon(side)
    local player, r = Game.GetPlayer(), active()
    if not player or not r then return say("Spawn Eddie first.") end
    if not ARMS or not GRIP_LINK[side] then return say("No arm parts: rerun build_glb.py and install_wolvenkit.py") end
    if not Game.FindEntityByID(r.id) then return say("Eddie isn't loaded here.") end
    local ok, item = pcall(copyOfVsWeapon, player)
    if not ok then return say("Couldn't read V's weapon: " .. tostring(item)) end
    if not item then return say("Draw a weapon first: Eddie gets a copy of the one in V's hand.") end
    local p, q = weaponPose(r, side)
    -- drops already lying around here, so the scan below doesn't mistake one for ours
    local before = {}
    pcall(function() for _, c in ipairs(dropsNear(p)) do before[c.key] = true end end)
    -- SpawnItemDrop drops an item out of the owner's inventory (vanilla vendors GiveItem first),
    -- so the copy goes into V's inventory and straight back out into the drop
    local ts = Game.GetTransactionSystem()
    local given
    ok, given = pcall(function() return ts:GiveItem(player, item, 1) end)
    if not ok or given == false then return say("Couldn't make a copy of the weapon: " .. tostring(given)) end
    local err
    ok, err = pcall(function()
        -- returns false even when the drop does appear, so its result isn't used: the drop is
        -- caught when it shows up (claimDrop), and the copy is taken back if it never does
        Game.GetLootManager():SpawnItemDrop(player, item, Vector4.new(p[1], p[2], p[3], 1),
            Quaternion.new(q[1], q[2], q[3], q[4]))
    end)
    if not ok then
        pcall(function() ts:RemoveItem(player, item, 1) end) -- don't leave V an extra copy
        return say("Couldn't spawn the weapon: " .. tostring(err))
    end
    releaseWeapon(r, side) -- a weapon already in that hand is let go
    r.weapons = r.weapons or {}
    local w = { id = nil, wait = 0, seen = false, item = item, at = p, scan = SCAN_EVERY, before = before }
    r.weapons[side] = w
    table.insert(pendingDrops, { w = w, pos = p })
    local a = armState(r)
    if a.target == "rest" then a.target = "reach" end -- hold it up and forward
    say(string.format("Eddie holds a copy of your weapon in its %s hand", HAND_NAMES[side]))
end

-- ---------------------------------------------------------------------------
-- Firing: the game's own NPC / turret firing (AIWeapon.Fire) on the gun in the gripper,
-- V as the owner like the sniper nest does it. The test button fires one shot along the
-- barrel and reports what it found.
-- ---------------------------------------------------------------------------
local function simTime()
    local ok, t = pcall(function() return EngineTime.ToFloat(Game.GetSimTime()) end)
    return ok and t or 0
end

local function triggerMode(weapon)
    local ok, mode = pcall(function()
        local rec = TweakDBInterface.GetWeaponItemRecord(ItemID.GetTDBID(weapon:GetItemID()))
        return rec:PrimaryTriggerMode():Type()
    end)
    if ok and mode then return mode, "weapon's own" end
    return gamedataTriggerMode.SemiAuto, "semi-auto (fallback)"
end

-- the first thing along a held gun's barrel (up to 60 m), or the point 60 m out
local function alongBarrel(p, q)
    local f = qrot(q, { 0, 1, 0 })
    local far = Vector4.new(p[1] + f[1] * 60, p[2] + f[2] * 60, p[3] + f[3] * 60, 1)
    local hit = raycast(Vector4.new(p[1], p[2], p[3], 1), far)
    if hit then return Vector4.new(hit.x, hit.y, hit.z, 1) end
    return far
end

local function fireOnce()
    local player, r = Game.GetPlayer(), active()
    if not player or not r then return say("Spawn Eddie first.") end
    local side, drop
    for _, s in ipairs({ "r", "l" }) do
        local w = r.weapons and r.weapons[s]
        drop = w and w.id and Game.FindEntityByID(w.id)
        if drop then side = s break end
    end
    if not drop then return say("Fire test: give Eddie a weapon first.") end
    local ok, weapon = pcall(function() return drop:GetItemObject() end)
    if not ok or not weapon then
        return say("Fire test: the drop has no weapon object (" .. tostring(weapon) .. ")")
    end
    local okA, attached = pcall(function() return weapon:IsAttached() end)
    local isWeapon = isA(weapon, "gameweaponObject")
    local mode, modeFrom = triggerMode(weapon)
    local at = alongBarrel(weaponPose(r, side))
    local okF, err = pcall(function()
        AIWeapon.Fire(player, weapon, simTime(), 1.0, mode, at)
    end)
    local p = weapon:GetWorldPosition()
    say(string.format("Fire test (%s hand): weapon object %s, IsAttached %s, trigger %s, along the barrel %.0f m: %s",
        HAND_NAMES[side], isWeapon and "is a WeaponObject" or "is NOT a WeaponObject",
        okA and tostring(attached) or "?", modeFrom,
        math.sqrt((at.x - p.x) ^ 2 + (at.y - p.y) ^ 2 + (at.z - p.z) ^ 2),
        okF and "AIWeapon.Fire called" or ("AIWeapon.Fire failed: " .. tostring(err))))
end

-- ---------------------------------------------------------------------------
-- Combat: when V hits a character, every Eddie holding a weapon turns to face it, swings
-- its guns onto it and fires at the weapon's own rate while it has a clear line of fire,
-- until the target is down, out of range or gone. Eddie's own hits (they count as V's)
-- don't pick new targets.
-- ---------------------------------------------------------------------------
local COMBAT_RANGE = 40           -- m
local AIM_CONE = math.rad(75)     -- the guns swing onto a target this far off Eddie's heading
local FIRE_CONE = math.rad(20)    -- and fire once the heading is this close
local AIM_RATE = 6                -- 1/s, how fast the guns swing onto / off the target
local CHEST = 1.2                 -- m above the target's feet
local SHOT_INTERVAL = 0.2         -- s between shots when the weapon doesn't say
local combatOn = true
local combatTarget = nil          -- { id } of the character V hit last
local combatAim = nil             -- {x, y, z} on it this frame

local function onPlayerHit(npc, evt)
    if not combatOn then return end
    local ad = evt.attackData
    local inst = ad and ad:GetInstigator()
    if not inst or not inst:IsPlayer() then return end
    local okW, wpn = pcall(function() return ad:GetWeapon() end)
    if okW and wpn then -- one of Eddie's shots: they count as V's too
        local key = entityKey(wpn)
        for _, rb in ipairs(robots) do
            for _, w in pairs(rb.weapons or {}) do
                if w.itemKey == key then return end
            end
        end
    end
    local okD, dead = pcall(function() return npc:IsDead() end)
    if okD and dead then return end
    local id = npc:GetEntityID()
    if combatTarget and entityKey(npc) == combatTarget.key then return end
    combatTarget = { id = id, key = entityKey(npc) }
    say("Eddie: engaging your target")
end

-- the target's aim point this frame (or nil), dropping it once it's down or gone
local function updateTarget()
    combatAim = nil
    if not combatTarget then return end
    local e = Game.FindEntityByID(combatTarget.id)
    if not e then
        combatTarget = nil
        return say("Eddie: target gone")
    end
    for _, check in ipairs({ function() return e:IsDead() end, function() return e:IsDefeated() end }) do
        local ok, yes = pcall(check)
        if ok and yes then
            combatTarget = nil
            return say("Eddie: target down")
        end
    end
    local p = e:GetWorldPosition()
    combatAim = { p.x, p.y, p.z + CHEST }
end

-- ---------------------------------------------------------------------------
-- Guard mode: Eddie finds its own targets (characters in combat with V, as the game
-- tracks it), nearest first and preferring ones it has a clear shot at, and drives to
-- them: follows V while it's quiet, closes in to GUARD_DISTANCE or until it has a line of
-- fire, but never further than GUARD_LEASH from V. V's own hits still pick the target.
-- ---------------------------------------------------------------------------
local GUARD_SCAN_EVERY = 0.5 -- s
local GUARD_DISTANCE = 10    -- m, closes in to this
local GUARD_LEASH = 20       -- m from V
local guardClock = 0

local function alive(e)
    for _, check in ipairs({ function() return e:IsDead() end, function() return e:IsDefeated() end }) do
        local ok, yes = pcall(check)
        if ok and yes then return false end
    end
    return true
end

local function fightingV(npc, player)
    local ok, yes = pcall(function() return NPCPuppet.IsInCombatWithTarget(npc, player) end)
    if ok then return yes == true end
    local okC, inCombat = pcall(function() return NPCPuppet.IsInCombat(npc) end)
    local okA, att = pcall(function() return npc:GetAttitudeTowards(player) end)
    return okC and inCombat == true and okA and tostring(att):find("Hostile") ~= nil
end

-- a clear shot from Eddie's gun height to the point?
local function lineOfFire(r, aim)
    local from = Vector4.new(r.pos.x, r.pos.y, r.pos.z + 0.7, 1)
    local hit = raycast(from, Vector4.new(aim[1], aim[2], aim[3], 1))
    if not hit then return true end
    local d = math.sqrt((aim[1] - from.x) ^ 2 + (aim[2] - from.y) ^ 2 + (aim[3] - from.z) ^ 2)
    return math.sqrt(hit.d) >= d - 0.5
end

local function armed(r)
    for _, w in pairs(r.weapons or {}) do
        if w.seen then return true end
    end
    return false
end

local function guardUpdate(dt)
    local r = active()
    if mode ~= "guard" or not r or not armed(r) then
        if combatTarget and combatTarget.auto then combatTarget = nil end
        return
    end
    guardClock = guardClock - dt
    if guardClock > 0 then return end
    guardClock = GUARD_SCAN_EVERY
    local player = Game.GetPlayer()
    if combatTarget then -- keep it while it's still a fight
        local e = Game.FindEntityByID(combatTarget.id)
        if e and alive(e) and (not combatTarget.auto or fightingV(e, player)) then return end
        combatTarget = nil
    end
    local best, bestScore = nil, math.huge
    for _, e in ipairs(entitiesAroundV(COMBAT_RANGE + GUARD_LEASH)) do
        if isA(e, "NPCPuppet") and alive(e) and fightingV(e, player) then
            local p = e:GetWorldPosition()
            local d = math.sqrt((p.x - r.pos.x) ^ 2 + (p.y - r.pos.y) ^ 2)
            if d < COMBAT_RANGE then
                local score = d + (lineOfFire(r, { p.x, p.y, p.z + CHEST }) and 0 or 15)
                if score < bestScore then best, bestScore = e, score end
            end
        end
    end
    if best then
        combatTarget = { id = best:GetEntityID(), key = entityKey(best), auto = true }
        say("Guard: engaging an enemy")
    end
end

-- where this Eddie should aim, if it's armed and the target is in range
local function engaged(r)
    if not combatOn or not combatAim or not armed(r) then return nil end
    local dx, dy = combatAim[1] - r.pos.x, combatAim[2] - r.pos.y
    if dx * dx + dy * dy > COMBAT_RANGE ^ 2 then return nil end
    return combatAim
end

-- how far (rad) a point is off Eddie's heading
local function offHeading(r, aim)
    local fx, fy = dirFromYaw(r.yaw)
    local dx, dy = aim[1] - r.pos.x, aim[2] - r.pos.y
    local n = math.sqrt(dx * dx + dy * dy)
    if n < 1e-6 then return 0 end
    return math.acos(math.max(-1, math.min(1, (fx * dx + fy * dy) / n)))
end

-- gun orientation with the barrel (+Y) from p at the aim point, upright (+Z up)
local function aimQuat(p, aim)
    local Y = { aim[1] - p[1], aim[2] - p[2], aim[3] - p[3] }
    local n = math.sqrt(Y[1] ^ 2 + Y[2] ^ 2 + Y[3] ^ 2)
    if n < 1e-6 then return nil end
    Y = { Y[1] / n, Y[2] / n, Y[3] / n }
    local Z = { -Y[3] * Y[1], -Y[3] * Y[2], 1 - Y[3] * Y[3] } -- up, minus its part along the barrel
    n = math.sqrt(Z[1] ^ 2 + Z[2] ^ 2 + Z[3] ^ 2)
    if n < 1e-6 then Z, n = { 1, 0, 0 }, 1 end
    Z = { Z[1] / n, Z[2] / n, Z[3] / n }
    local X = { Y[2] * Z[3] - Y[3] * Z[2], Y[3] * Z[1] - Y[1] * Z[3], Y[1] * Z[2] - Y[2] * Z[1] }
    return quatFromAxes(X, Y, Z)
end

local function tryFire(w, drop, p, aim, dt)
    w.cool = (w.cool or 0) - dt
    if w.cool > 0 then return end
    local from, to = Vector4.new(p[1], p[2], p[3], 1), Vector4.new(aim[1], aim[2], aim[3], 1)
    local hit = raycast(from, to)
    local dist = math.sqrt((aim[1] - p[1]) ^ 2 + (aim[2] - p[2]) ^ 2 + (aim[3] - p[3]) ^ 2)
    if hit and math.sqrt(hit.d) < dist - 0.5 then return end -- a wall in the way
    local weapon = drop:GetItemObject()
    if not weapon then return end
    local now = simTime()
    AIWeapon.Fire(Game.GetPlayer(), weapon, now, 1.0, triggerMode(weapon), to,
        combatTarget and Game.FindEntityByID(combatTarget.id) or nil)
    local ok, nextShot = pcall(function() return AIWeapon.GetNextShotTimeStamp(weapon) end)
    local gap = ok and nextShot and nextShot - now or 0
    w.cool = (gap > 0.02 and gap < 3) and gap or SHOT_INTERVAL
end

local function placeWeapons(r, dt)
    if not r.weapons then return end
    for side, w in pairs(r.weapons) do
        if not w.id then
            w.scan = w.scan - dt
            if w.scan <= 0 then
                w.scan = SCAN_EVERY
                local taken = {}
                for _, rb in ipairs(robots) do
                    for _, other in pairs(rb.weapons or {}) do
                        if other.key then taken[other.key] = true end
                    end
                end
                local ok, err = pcall(scanForDrop, w, w.at, taken)
                if not ok and not w.scanErr then
                    w.scanErr = true
                    print("[eddie_spawner] drop scan failed: " .. tostring(err))
                end
                if w.id then forgetPending(w) end
            end
        end
        local e = w.id and Game.FindEntityByID(w.id)
        if e then
            w.seen = true
            if not w.itemKey then -- to tell Eddie's own hits from V's
                local ok, item = pcall(function() return e:GetItemObject() end)
                if ok and item then w.itemKey = entityKey(item) end
            end
            -- the drop object; the weapon you see is attached to it and follows
            local p, q = weaponPose(r, side)
            -- in a fight: swing the gun from the gripper's direction onto the target
            local aim = engaged(r)
            local off = aim and offHeading(r, aim) or math.pi
            local goal = off < AIM_CONE and 1 or 0
            w.blend = w.blend or 0
            w.blend = goal > w.blend and math.min(goal, w.blend + AIM_RATE * dt) or math.max(goal, w.blend - AIM_RATE * dt)
            if aim then w.lastAim = aim end
            local qa = w.blend > 0 and w.lastAim and aimQuat(p, w.lastAim)
            if qa then q = slerp(q, qa, w.blend) end
            Game.GetTeleportationFacility():Teleport(e, Vector4.new(p[1], p[2], p[3], 1), toEuler(q))
            if aim and off < FIRE_CONE and w.blend >= 1 then tryFire(w, e, p, aim, dt) end
        elseif w.seen then
            dropWeapon(r, side)
            say(string.format("The weapon in Eddie's %s hand is gone (picked up?)", HAND_NAMES[side]))
        else
            w.wait = w.wait + dt
            if w.wait > WEAPON_SPAWN_WAIT then
                dropWeapon(r, side)
                -- if it never left V's inventory, take the copy back
                local player = Game.GetPlayer()
                if player and w.item then
                    pcall(function() Game.GetTransactionSystem():RemoveItem(player, w.item, 1) end)
                end
                say(w.id and "The weapon didn't show up in the world."
                    or "Couldn't catch the weapon drop (it may be lying on the floor).")
            end
        end
    end
end

-- desired world velocity and heading for this frame
-- keep FOLLOW_DISTANCE from V, looking at her
local function followV(r)
    local p = Game.GetPlayer():GetWorldPosition()
    local dx, dy = p.x - r.pos.x, p.y - r.pos.y
    local dist = math.sqrt(dx * dx + dy * dy)
    local s = speed * math.max(0, math.min(1, (dist - FOLLOW_DISTANCE) / 1.0))
    if dist > 1e-3 then return dx / dist * s, dy / dist * s, yawFromDir(dx, dy) end
    return 0, 0, nil
end

local function command(r, dt)
    local vx, vy, faceYaw, turn = 0, 0, nil, 0
    if mode == "follow" then
        vx, vy, faceYaw = followV(r)
    elseif mode == "guard" then
        local aim = engaged(r)
        local p = Game.GetPlayer():GetWorldPosition()
        local toV = math.sqrt((p.x - r.pos.x) ^ 2 + (p.y - r.pos.y) ^ 2)
        if aim and toV < GUARD_LEASH then
            -- close in until it's near enough and has a clear shot
            local dx, dy = aim[1] - r.pos.x, aim[2] - r.pos.y
            local d = math.sqrt(dx * dx + dy * dy)
            local want = lineOfFire(r, aim) and GUARD_DISTANCE or 2.0
            local s = speed * math.max(0, math.min(1, (d - want) / 1.0))
            if d > 1e-3 then vx, vy = dx / d * s, dy / d * s end
        else
            vx, vy, faceYaw = followV(r) -- quiet, or too far from V: back to her
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
    -- in a fight Eddie faces its target, whatever it's driving (when ridden, V steers)
    local aim = engaged(r)
    if aim and mode ~= "ride" then
        faceYaw, turn = yawFromDir(aim[1] - r.pos.x, aim[2] - r.pos.y), 0
    end
    return vx, vy, faceYaw, turn
end

local function drive(dt)
    local r = active()
    if not r or not riding and not engaged(r) and mode == "idle" and r.vel.x == 0 and r.vel.y == 0 then return end
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
    local okDrop, errDrop = pcall(ObserveAfter or Observe, "gameItemDropObject", "OnGameAttached", function(self)
        pcall(claimDrop, self)
    end)
    local okHit, errHit = pcall(ObserveAfter or Observe, "NPCPuppet", "OnHit", function(self, evt)
        pcall(onPlayerHit, self, evt)
    end)
    if not okHit then print("[eddie_spawner] no combat (can't watch hits): " .. tostring(errHit)) end
    print("[eddie_spawner] loaded" .. (ok and "" or " (no ride controls: " .. tostring(err) .. ")")
        .. (okDrop and "" or " (no weapon holding: " .. tostring(errDrop) .. ")"))
end)
registerForEvent("onShutdown", function()
    if riding then pcall(setRestriction, false) end
end)
registerForEvent("onOverlayOpen", function() overlayOpen = true end)
registerForEvent("onOverlayClose", function() overlayOpen = false end)

registerForEvent("onUpdate", function(dt)
    if #robots == 0 and fallingCount() == 0 or not Game.GetPlayer() then return end
    local srh = Game.GetSystemRequestsHandler()
    if srh:IsPreGame() or srh:IsGamePaused() then return end
    if ride.getOff then stopRiding() end
    pcall(guardUpdate, math.min(dt, 0.1))
    pcall(updateTarget)
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
    -- every Eddie keeps hold of its weapons, not only the driven one
    for _, rb in ipairs(robots) do
        if rb.weapons and Game.FindEntityByID(rb.id) then
            ok, err = pcall(placeWeapons, rb, math.min(dt, 0.1))
            if not ok then
                rb.weapons = nil
                say("Weapon error: " .. tostring(err))
            end
        end
    end
    ok, err = pcall(updateFalling, math.min(dt, 0.1))
    if not ok then
        falling = {}
        say("Weapon error: " .. tostring(err))
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
        if ImGui.Button("Guard") then setMode("guard") end
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
            local closed = r and (holding(r) or r.arm and r.arm.grip > 0)
            if ImGui.Button(closed and "Open grippers" or "Close grippers") then toggleGrip() end
            ImGui.Text("Weapons (a copy of the one V has drawn)")
            if ImGui.Button("Give: right hand") then giveWeapon("r") end
            ImGui.SameLine()
            if ImGui.Button("Give: left hand") then giveWeapon("l") end
            ImGui.SameLine()
            if ImGui.Button("Drop weapons") then dropWeapons(r) end
            ImGui.SameLine()
            if ImGui.Button("Fire once (test)") then fireOnce() end
            combatOn = ImGui.Checkbox("Shoot what I shoot", combatOn)
            if not combatOn then combatTarget = nil end
        end
        if status ~= "" then ImGui.TextWrapped(status) end
    end
    ImGui.End()
end)

registerHotkey("spawn_eddie", "Spawn Eddie in front of V", spawn)
registerHotkey("despawn_eddie", "Despawn all Eddies", despawnAll)
registerHotkey("eddie_follow", "Follow V on/off", function() setMode(mode == "follow" and "idle" or "follow") end)
registerHotkey("eddie_guard", "Guard on/off (follows V, fights enemies on its own)",
    function() setMode(mode == "guard" and "idle" or "guard") end)
registerHotkey("eddie_goto", "Drive to where V looks", sendToLook)
registerHotkey("eddie_stop", "Stop", function() setMode("idle") end)
registerHotkey("eddie_ride", "Ride Eddie / get off", toggleRide)
for _, p in ipairs(POSE_LABELS) do
    registerHotkey("eddie_arms_" .. p[1], "Arms: " .. p[2], function() setArms(p[1]) end)
end
registerHotkey("eddie_grip", "Grippers open / close", toggleGrip)
registerHotkey("eddie_weapon_right", "Give Eddie your weapon (right hand)", function() giveWeapon("r") end)
registerHotkey("eddie_weapon_left", "Give Eddie your weapon (left hand)", function() giveWeapon("l") end)
registerHotkey("eddie_weapon_drop", "Eddie drops its weapons", function() dropWeapons(active()) end)
registerHotkey("eddie_weapon_fire_test", "Eddie fires once (test)", fireOnce)
registerHotkey("eddie_combat", "Shoot what I shoot on / off", function()
    combatOn = not combatOn
    if not combatOn then combatTarget = nil end
    say(combatOn and "Eddie shoots what you shoot" or "Eddie holds fire")
end)

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
