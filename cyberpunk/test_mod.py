"""Run the eddie_spawner CET mod against a mocked CET + game API and check spawning,
driving, the castors and wheels, riding and the arms. No game needed.

Run:  .venv/Scripts/python test_mod.py      (after build_glb.py and install_wolvenkit.py,
                                              which write out/ and eddie_parts.lua)
Each check runs for both possible yaw conventions of the game.
"""
import math
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
MOD = HERE.parent / "wolvenkit/source/resources/bin/x64/plugins/cyber_engine_tweaks/mods/eddie_spawner"
PHYSICS = HERE / "out" / "eddie_physics.json"

if len(sys.argv) < 2:  # run once per yaw convention
    failed = [s for s in ("1", "-1") if subprocess.run([sys.executable, __file__, s]).returncode]
    sys.exit(1 if failed else 0)

from lupa import LuaRuntime  # noqa: E402

SRC = (MOD / "init.lua").read_text(encoding="utf-8")
YAW_SIGN = int(sys.argv[1])  # the mocked game's yaw convention
print(f"== game yaw convention {YAW_SIGN:+d}")

MOCK = r"""
local S = ...
local world = { wall_x = 5.0 }
local player = { x = 0, y = 0, z = 0, yaw = 30 }
local cam = { x = 0, y = 0, z = -1 }
local teleports = { n = 0 }
local entities = {}
local weapon = { held = true, record = "Items.Preset_Lexington_Default", delay = 3 }
local drops = {}
local pendingDrops = {}
local inventory = { n = 0 } -- items given to V that she still has (not counting her own gear)
local lootRefuses = false
local hookOff = false
local nextId = 0
local function newId() nextId = nextId + 1; return nextId end
TweakDBID = { new = function(s) return { tdb = s } end }
ItemID = { FromTDBID = function(t) return { id = t, fresh = true } end, GetTDBID = function(i) return i.id end }
function GetSingleton(name)
  assert(name == "Quaternion")
  return { ToEulerAngles = function(_, q) return { roll = 0, pitch = 0, yaw = 0, _q = q } end }
end
handlers = { events = {}, hotkeys = {}, inputs = {}, observers = {} }

Vector4 = { new = function(x, y, z, w) return { x = x, y = y, z = z, w = w } end }
Quaternion = { new = function(i, j, k, r) return { i = i, j = j, k = k, r = r } end }
CName = { new = function(s) return "cn:" .. s end }
local Ent = {}
Ent.__index = Ent
function Ent:FindComponentByName(n)
  assert(type(n) == "string" and n:sub(1, 3) == "cn:", "expects a CName")
  self.comps = self.comps or {}
  local c = self.comps[n]
  if not c then
    c = { SetLocalTransform = function(c, p, q) c.p = p; c.q = q; c.n = (c.n or 0) + 1 end }
    self.comps[n] = c
  end
  return c
end
EulerAngles = { new = function(r, p, y) return { roll = r, pitch = p, yaw = y } end }
function Ent:IsA(c) if c == "NPCPuppet" then return self.isNPC == true end return (c == "gameItemDropObject" and self.isDrop) or (c == "gameItemObject" and self.isItem) or (c == "gameweaponObject" and self.isItem) or false end
function Ent:GetWorldPosition() return self.pos end
function Ent:GetEntityID() return self.eid end
function Ent:GetItemObject() return self.itemObj end
function Ent:GetConnectedItemDrop() return self.drop end
function Ent:IsAttached() return true end
function Ent:IsDead() return self.dead or false end
function Ent:IsDefeated() return false end
-- in combat with V, as the game tracks it
NPCPuppet = { IsInCombatWithTarget = function(npc, player) return npc.fighting == true end }
-- a character V (or Eddie) can shoot at
function spawnNPC(x, y, z)
  local id = newId()
  entities[id] = setmetatable({ pos = Vector4.new(x, y, z, 1), eid = id, isNPC = true }, Ent)
  return entities[id]
end
function makeGun(eid) return setmetatable({ eid = eid }, Ent) end
-- one hit on npc by V's attack with `weapon` (V's own gun, or one of Eddie's)
function hitBy(npc, weapon)
  local cb = handlers.observers["NPCPuppet.OnHit"]
  local ad = { GetInstigator = function() return Game.GetPlayer() end, GetWeapon = function() return weapon end }
  if cb then cb(npc, { attackData = ad }) end
end
function Ent:GetItemID() return self.itemID end
local fires = {}
AIWeapon = { Fire = function(owner, weapon, t, tbh, mode, pos, target)
  fires[#fires + 1] = { owner = owner, weapon = weapon, t = t, tbh = tbh, mode = mode, pos = pos, target = target } end,
  GetNextShotTimeStamp = function(weapon) return 12.5 + 0.1 end } -- sim time is 12.5: a shot every 0.1 s
EngineTime = { ToFloat = function(t) return t.seconds end }
TweakDBInterface = { GetWeaponItemRecord = function(id)
  return { PrimaryTriggerMode = function() return { Type = function() return "FullAuto" end } end } end }
gamedataTriggerMode = { SemiAuto = "SemiAuto" }

local function fwd(yaw) local r = math.rad(yaw) return S * math.sin(r), math.cos(r) end

local P = {}
function P:GetWorldPosition() return Vector4.new(player.x, player.y, player.z, 1) end
function P:GetWorldForward() local x, y = fwd(player.yaw) return Vector4.new(x, y, 0, 0) end
function P:GetWorldYaw() return player.yaw end
P.isPlayer = true
function P:IsPlayer() return true end
function P:GetEntityID() return "pid" end
function P:GetRecordID() return "prec" end
function P:RegisterInputListener(_, a) end
function P:UnregisterInputListener(_, a) end
local effects = {}
local ses = {
  ApplyStatusEffect = function(_, id, e, src, inst) effects[e] = true end,
  RemoveStatusEffect = function(_, id, e) effects[e] = nil end,
}
function Observe(cls, fn, cb) handlers.observers[cls .. "." .. fn] = cb end
ObserveAfter = Observe
-- like the game: SpawnItemDrop only says whether it worked; the drop object shows up a few
-- frames later, firing gameItemDropObject:OnGameAttached
-- the drop object, and the weapon you see as its own ItemObject connected to it
local function attachDrop(pos, item)
  local id = newId()
  local drop = setmetatable({ pos = pos, item = item, eid = id, isDrop = true }, Ent)
  drop.itemObj = setmetatable({ pos = pos, isItem = true, drop = drop, itemID = item, eid = "item" .. id }, Ent)
  entities[id] = drop
  drops[#drops + 1] = id
  local cb = handlers.observers["gameItemDropObject.OnGameAttached"]
  if cb and not hookOff then cb(drop) end
  return id
end
gameTargetSearchQuery = { new = function() return {} end }
gameTargetingSet = { Complete = 3 }
gametargetingTargetPartInfo = { GetComponent = function(part) return part end }
function tick()
  for i = #pendingDrops, 1, -1 do
    local d = pendingDrops[i]
    d.delay = d.delay - 1
    if d.delay <= 0 then table.remove(pendingDrops, i); attachDrop(d.pos, d.item) end
  end
end
ListenerAction = {
  GetName = function(a) return a.name end,
  GetType = function(a) return { value = a.kind } end,
  GetValue = function(a) return a.value end,
}
function P:GetWorldTransform()
  local t = {}
  function t:SetPosition(p) self.pos = p end
  function t:SetOrientationEuler(e) self.rot = e end
  return t
end

local sqs = {}
-- flat floor at z=0; a wall plane at x = wall_x (for horizontal rays only)
function sqs:SyncRaycastByCollisionGroup(a, b, group, s, d)
  if group ~= "Static" then return false, nil end
  if a.z > 0 and b.z < 0 then
    local t = a.z / (a.z - b.z)
    return true, { position = { x = a.x + (b.x - a.x) * t, y = a.y + (b.y - a.y) * t, z = 0 } }
  end
  if a.x < world.wall_x and b.x >= world.wall_x then
    local t = (world.wall_x - a.x) / (b.x - a.x)
    return true, { position = { x = world.wall_x, y = a.y + (b.y - a.y) * t, z = a.z + (b.z - a.z) * t } }
  end
  return false, nil
end

Game = {
  GetPlayer = function() return setmetatable({}, { __index = P }) end,
  GetSpatialQueriesSystem = function() return sqs end,
  GetCameraSystem = function() return { GetActiveCameraForward = function() return Vector4.new(cam.x, cam.y, cam.z, 0) end } end,
  GetSystemRequestsHandler = function() return { IsPreGame = function() return false end, IsGamePaused = function() return false end } end,
  FindEntityByID = function(id)
    assert(type(id) ~= "boolean", "FindEntityByID needs an EntityID")
    return entities[id]
  end,
  GetTransactionSystem = function() return {
    GetItemInSlot = function(_, owner, slot)
      if weapon.held and owner.isPlayer and slot.tdb == "AttachmentSlots.WeaponRight" then
        return { GetItemID = function() return { id = TweakDBID.new(weapon.record), seed = 7 } end }
      end end,
    GiveItem = function(_, owner, item, n)
      assert(owner.isPlayer); inventory[item] = (inventory[item] or 0) + n; inventory.n = inventory.n + n; return true end,
    RemoveItem = function(_, owner, item, n)
      if (inventory[item] or 0) < n then return false end
      inventory[item] = inventory[item] - n; inventory.n = inventory.n - n; return true end,
  } end,
  -- like the game: only drops an item the owner has, and takes it out of their inventory
  GetLootManager = function() return { SpawnItemDrop = function(_, owner, item, pos, rot)
    if lootRefuses or (inventory[item] or 0) < 1 then return false end
    inventory[item] = inventory[item] - 1; inventory.n = inventory.n - 1
    pendingDrops[#pendingDrops + 1] = { pos = pos, item = item, delay = weapon.delay }
    return true end } end,
  GetTeleportationFacility = function() return { Teleport = function(_, e, p, r)
    if e.isPlayer then player.x, player.y, player.z, player.yaw = p.x, p.y, p.z, r.yaw; return end
    teleports.n = teleports.n + 1; e.pos = p; e.yaw = r.yaw; e.rot = r; e.tpn = (e.tpn or 0) + 1 end } end,
  GetStatusEffectSystem = function() return ses end,
  GetSimTime = function() return { seconds = 12.5 } end,
  -- every drop's weapon object around V (the targeting query's view of them)
  GetTargetingSystem = function() return { GetTargetParts = function(_, player, q)
    local parts = {}
    for _, e in pairs(entities) do
      if e.isDrop then parts[#parts + 1] = { GetEntity = function() return e.itemObj end } end
      if e.isNPC then parts[#parts + 1] = { GetEntity = function() return e end } end
    end
    return true, parts end } end,
  NameToString = function(n) return n end,
}
exEntitySpawner = {
  Spawn = function(path, t, app) local id = newId(); entities[id] = setmetatable({ pos = t.pos, yaw = t.rot.yaw }, Ent); return id end,
  Despawn = function(e) end,
}
function registerForEvent(n, f) handlers.events[n] = f end
function registerHotkey(id, d, f) handlers.hotkeys[id] = f end
function registerInput(id, d, f) handlers.inputs[id] = f end
ImGui = {}
return { world = world, player = player, cam = cam, entities = entities, teleports = teleports, effects = effects,
         weapon = weapon, drops = drops, attachDrop = attachDrop, inventory = inventory,
         fires = fires, setLootRefuses = function(v) lootRefuses = v end, setHookOff = function(v) hookOff = v end }
"""

L = LuaRuntime(unpack_returned_tuples=True)
L.execute("LOG = {}; print = function(...) LOG[#LOG + 1] = table.concat({...}, ' ') end")  # keep mod prints quiet
STATUS = lambda: " | ".join(L.globals().LOG.values())

L.execute(f'package.path = "{MOD.as_posix()}/?.lua;" .. package.path')
env = L.execute(MOCK, YAW_SIGN)
L.execute(SRC)
H = L.globals().handlers
H.events["onInit"]()
TICK = L.globals().tick


def step(secs):
    for _ in range(int(secs * 60)):
        TICK()
        H.events["onUpdate"](1 / 60)

ent = lambda: env.entities[1]
pl = env.player


def dist(ax, ay, bx, by):
    return math.hypot(ax - bx, ay - by)


def yaw_to(dx, dy):
    return math.degrees(math.atan2(YAW_SIGN * dx, dy))


def angdiff(a, b):
    return abs((a - b + 180) % 360 - 180)


ok = True
def check(name, cond, info=""):
    global ok
    ok &= bool(cond)
    print(f"{'PASS' if cond else 'FAIL'}  {name}  {info}")


# spawn 2 m in front of V, facing V
H.hotkeys["spawn_eddie"]()
fx, fy = YAW_SIGN * math.sin(math.radians(30)), math.cos(math.radians(30))
e = ent()
check("spawn position", dist(e.pos.x, e.pos.y, 2 * fx, 2 * fy) < 1e-6, f"({e.pos.x:.2f},{e.pos.y:.2f})")
check("spawn faces V", angdiff(e.yaw, yaw_to(-fx, -fy)) < 1e-6, f"yaw {e.yaw:.1f}")

# follow: V walks to (0, -4) -> stays ~2 m away, facing V
H.hotkeys["eddie_follow"]()
pl.x, pl.y = 0, -4
step(10)
e = ent()
d = dist(e.pos.x, e.pos.y, pl.x, pl.y)
check("follow keeps ~2 m", 1.9 < d < 2.3, f"d={d:.2f}")
check("follow faces V", angdiff(e.yaw, yaw_to(pl.x - e.pos.x, pl.y - e.pos.y)) < 2, f"yaw {e.yaw:.1f}")
check("stays on floor", abs(e.pos.z) < 1e-9)

# go where V looks: V at (0,-4), eye 1.6 m, looking down 45 deg along +y -> floor hit at (0,-2.4)
H.hotkeys["eddie_stop"]()
s = math.sqrt(0.5)
env.cam.x, env.cam.y, env.cam.z = 0, s, -s
H.hotkeys["eddie_goto"]()
step(10)
e = ent()
check("goto arrives", dist(e.pos.x, e.pos.y, 0, -4 + 1.6) < 0.12, f"({e.pos.x:.2f},{e.pos.y:.2f})")

# manual: forward 1 s along heading, then turn left 0.5 s
y0, x0, yy0 = ent().yaw, ent().pos.x, ent().pos.y
H.inputs["eddie_fwd"](True); step(1.0); H.inputs["eddie_fwd"](False); step(1.5)
e = ent()
mx, my = e.pos.x - x0, e.pos.y - yy0
hx, hy = YAW_SIGN * math.sin(math.radians(y0)), math.cos(math.radians(y0))
along = mx * hx + my * hy
check("manual forward moves along heading", along > 0.5 and abs(mx * hy - my * hx) < 1e-6, f"along={along:.2f}")

H.inputs["eddie_turn_left"](True); step(0.5); H.inputs["eddie_turn_left"](False)
e = ent()
# left turn = counter-clockwise seen from above: heading vector rotates CCW
nx, ny = YAW_SIGN * math.sin(math.radians(e.yaw)), math.cos(math.radians(e.yaw))
cross = hx * ny - hy * nx
check("turn left is counter-clockwise", cross > 0.5, f"cross={cross:.2f} yaw {y0:.0f}->{e.yaw:.0f}")
sx, sy = ent().pos.x, ent().pos.y
H.inputs["eddie_right"](True); step(1.0); H.inputs["eddie_right"](False); step(1.5)
e2 = ent()
rx, ry = ny, -nx  # right of heading
mv = (e2.pos.x - sx) * rx + (e2.pos.y - sy) * ry
check("strafe right moves to the right", mv > 0.4, f"{mv:.2f} from ({e.pos.x:.2f},{e.pos.y:.2f}) to ({e2.pos.x:.2f},{e2.pos.y:.2f}) right=({rx:.2f},{ry:.2f})")

# --- castors and wheels (component transforms are in entity space: front +Y, right +X)
UNITS = ("fl", "rl", "rr", "fr")
comp = lambda n: ent().comps[f"cn:eddie_part_{n}"]


def trail(u):
    """wheel-pair midpoint minus pivot, entity space"""
    p, a, b = comp(u).p, comp(f"{u}_r").p, comp(f"{u}_l").p
    return (a.x + b.x) / 2 - p.x, (a.y + b.y) / 2 - p.y


def qrot(q, v):
    i, j, k, r = q.i, q.j, q.k, q.r
    x, y, z = v
    # v' = q v q*
    tx, ty, tz = 2 * (j * z - k * y), 2 * (k * x - i * z), 2 * (i * y - j * x)
    return (x + r * tx + (j * tz - k * ty), y + r * ty + (k * tx - i * tz), z + r * tz + (i * ty - j * tx))


H.inputs["eddie_fwd"](True); step(1.0)
tr = [trail(u) for u in UNITS]
check("forward: castors trail behind the pivots", all(t[1] < -0.009 and abs(t[0]) < 0.002 for t in tr),
      " ".join(f"({x:+.3f},{y:+.3f})" for x, y in tr))
# rolling forward at v: wheel turns at v/R about the axle, the sense that moves its top forward
R = 0.05276


def qmul(a, b):
    return (a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
            a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
            a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
            a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2])


def qt(q):
    return (q.i, q.j, q.k, q.r)


q0 = {u: qt(comp(f"{u}_{s}").q) for u in UNITS for s in "rl" for u in [f"{u}_{s}"]} if False else     {f"{u}_{s}": qt(comp(f"{u}_{s}").q) for u in UNITS for s in "rl"}
step(1 / 60)
rates = []
for n, a in q0.items():
    b = qt(comp(n).q)
    d = qmul(b, (-a[0], -a[1], -a[2], a[3]))  # b * a^-1, rotation done this frame (entity space)
    if d[3] < 0:
        d = tuple(-x for x in d)
    s = math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)
    ang = 2 * math.atan2(s, d[3])
    axis = [x / s for x in d[:3]] if s > 1e-12 else [0, 0, 0]
    # omega x up must point forward (+Y): omega along -X
    rates.append((n, ang * 60 * -axis[0]))
check("forward: wheels roll forward at v/R", all(abs(w - 1.0 / R) < 0.02 / R for _, w in rates),
      " ".join(f"{n}:{w:.1f}" for n, w in rates) + f" (v/R = {1 / R:.1f} rad/s)")
H.inputs["eddie_fwd"](False); step(1.5)

H.inputs["eddie_right"](True); step(1.0)
tr = [trail(u) for u in UNITS]
check("strafe right: castors trail to the left", all(t[0] < -0.009 and abs(t[1]) < 0.002 for t in tr),
      " ".join(f"({x:+.3f},{y:+.3f})" for x, y in tr))
H.inputs["eddie_right"](False); step(1.5)

H.inputs["eddie_turn_left"](True); step(1.0)
ok_turn = True
info = []
for u in UNITS:
    p = comp(u).p
    vx, vy = -p.y, p.x  # counter-clockwise: velocity of the pivot = w x p
    tx, ty = trail(u)
    cosang = (tx * vx + ty * vy) / math.hypot(tx, ty) / math.hypot(vx, vy)
    ok_turn &= cosang < -0.99
    info.append(f"{u}:{cosang:+.3f}")
check("turn in place: castors trail their pivots' circle", ok_turn, " ".join(info))
H.inputs["eddie_turn_left"](False); step(1.5)
n_before = comp("fl_r").n
step(1.0)
check("parked: parts not touched", comp("fl_r").n == n_before)

# wall at x=5: send it toward x=8 -> stops before the wall
H.hotkeys["eddie_stop"]()
pl.x, pl.y = 0, ent().pos.y
env.cam.x, env.cam.y, env.cam.z = 0.95, 0, -0.312  # hits floor ~4.9 m away... aim beyond: move eye
pl.x = ent().pos.x
env.cam.x, env.cam.y, env.cam.z = 0.99, 0, -0.14
H.hotkeys["eddie_goto"]()
step(15)
e = ent()
check("stops before wall", e.pos.x < 5 - 0.5 and e.pos.x > 3.5, f"x={e.pos.x:.2f}")

# drop: floor ends -> modelled as no floor beyond y > 50 is not tested; teleports happened
check("teleport used", env.teleports.n > 100, str(env.teleports.n))

# --- riding
act = lambda name, kind, value=0.0: H.observers["PlayerPuppet.OnAction"](None, L.table_from({"name": name, "kind": kind, "value": value}))
locked = lambda: bool(env.effects["GameplayRestriction.NoMovement"])
H.hotkeys["eddie_stop"](); step(2)
e = ent()
pl.x, pl.y, pl.z = e.pos.x - 20, e.pos.y, 0
H.hotkeys["eddie_ride"]()
check("ride: too far away refused", not locked())
pl.x, pl.yaw = e.pos.x - 2, yaw_to(-1, 0)  # V looks along -x
H.hotkeys["eddie_ride"](); step(1 / 60)
e = ent()
hx, hy = YAW_SIGN * math.sin(math.radians(e.yaw)), math.cos(math.radians(e.yaw))
spot = (e.pos.x - 0.185 * hx, e.pos.y - 0.185 * hy, e.pos.z + 0.93)
check("ride: V's walking locked", locked())
check("ride: V on top of the torso", dist(pl.x, pl.y, spot[0], spot[1]) < 1e-6 and abs(pl.z - spot[2]) < 1e-6,
      f"V ({pl.x:.2f},{pl.y:.2f},{pl.z:.2f}) spot ({spot[0]:.2f},{spot[1]:.2f},{spot[2]:.2f})")
step(2)
check("ride: Eddie turns to face V's view", angdiff(ent().yaw, pl.yaw) < 0.5, f"{ent().yaw:.1f} vs {pl.yaw:.1f}")
y_before = pl.yaw
act("CameraMouseX", "RELATIVE_CHANGE", -35 * 10.0); step(1 / 60)
d = (pl.yaw - y_before + 180) % 360 - 180
check("ride: mouse turns V (+10 deg = counter-clockwise)", abs(d - 10) < 1e-6, f"{d:.2f}")
step(1 / 60)
check("ride: mouse delta used once", abs(((pl.yaw - y_before + 180) % 360 - 180) - 10) < 1e-6)
act("CameraMouseX", "RELATIVE_CHANGE", 35 * 10.0); step(2)
x0, y0 = ent().pos.x, ent().pos.y
act("Forward", "BUTTON_PRESSED"); step(2.0); act("Forward", "BUTTON_RELEASED"); step(1.5)
e = ent()
check("ride: W drives where V looks (-x)", e.pos.x - x0 < -1.5 and abs(e.pos.y - y0) < 1e-3, f"moved ({e.pos.x - x0:.2f},{e.pos.y - y0:.2f})")
hx, hy = YAW_SIGN * math.sin(math.radians(e.yaw)), math.cos(math.radians(e.yaw))
check("ride: V carried along", dist(pl.x, pl.y, e.pos.x - 0.185 * hx, e.pos.y - 0.185 * hy) < 1e-6)
x0, y0 = e.pos.x, e.pos.y
act("Right", "BUTTON_PRESSED"); step(1.0); act("Right", "BUTTON_RELEASED"); step(1.5)
e = ent()
fx, fy = YAW_SIGN * math.sin(math.radians(pl.yaw)), math.cos(math.radians(pl.yaw))
mv = (e.pos.x - x0) * fy + (e.pos.y - y0) * -fx
check("ride: D strafes right of the view", mv > 0.4, f"{mv:.2f}")
H.hotkeys["eddie_follow"](); step(0.5)
check("ride: follow refused while riding", locked())
H.hotkeys["eddie_goto"]()
act("MoveY", "AXIS_CHANGE", 0.8); step(1 / 60)
act("MoveY", "AXIS_CHANGE", 0.0); step(1.5)
act("Jump", "BUTTON_PRESSED"); step(1 / 60); act("Jump", "BUTTON_RELEASED")
e = ent()
d = dist(pl.x, pl.y, e.pos.x, e.pos.y)
check("ride: Jump gets off next to Eddie, on the floor", not locked() and abs(pl.z) < 1e-9 and 0.9 < d < 1.1, f"d={d:.2f} z={pl.z:.2f}")
x0, y0 = e.pos.x, e.pos.y
act("Forward", "BUTTON_PRESSED"); step(1.0); act("Forward", "BUTTON_RELEASED")
check("ride: W ignored after getting off", dist(ent().pos.x, ent().pos.y, x0, y0) < 1e-9)
pl.x, pl.y = e.pos.x + 1, e.pos.y
H.hotkeys["eddie_ride"](); step(0.2)
H.hotkeys["despawn_eddie"]()
check("ride: despawn gives V's walking back", not locked())

# --- arms
import json as _json
sys.path.insert(0, str(HERE))
import numpy as np
from scipy.spatial.transform import Rotation
import build_glb as B
ARMS = _json.loads(PHYSICS.read_text())["moving_parts"]["arms"]
H.hotkeys["spawn_eddie"]()
E2 = env.entities[len(env.entities)]
check("arms: untouched until asked", not any(k.startswith("cn:eddie_part_l_") or k.startswith("cn:eddie_part_r_")
                                             for k in (E2.comps or {}).keys()) if E2.comps else True)

def ent_space(T):
    p = T[:3, 3]
    q = (Rotation.from_euler("z", 90, degrees=True) * Rotation.from_matrix(T[:3, :3])).as_quat()
    return (-p[1], p[0], p[2]), q

def arm_err(qvals):
    worst_p, worst_q = 0.0, 0.0
    for e, T in zip(ARMS["links"], B.arm_fk(ARMS["links"], qvals)):
        c = E2.comps["cn:eddie_part_" + e["name"]]
        p, q = ent_space(T)
        worst_p = max(worst_p, math.dist((c.p.x, c.p.y, c.p.z), p))
        worst_q = max(worst_q, 1 - abs(np.dot([c.q.i, c.q.j, c.q.k, c.q.r], q)))
    return worst_p, worst_q

def pose_q(name, grip=0.0):
    return dict(ARMS["vars"], **ARMS["poses"][name], lg=grip, rg=grip)

def rest_place_err():
    """rest pose from Lua FK vs the rest placement baked into the .ent"""
    worst = 0.0
    for e in ARMS["links"]:
        c = E2.comps["cn:eddie_part_" + e["name"]]
        T = np.eye(4); T[:3, :3] = Rotation.from_quat(e["rest"][3:]).as_matrix(); T[:3, 3] = e["rest"][:3]
        p, _ = ent_space(T)
        worst = max(worst, math.dist((c.p.x, c.p.y, c.p.z), p))
    return worst

H.hotkeys["eddie_arms_reach"](); step(0.5)
p_mid, _ = arm_err(pose_q("reach"))
check("arms: move gradually", p_mid > 0.01, f"{p_mid:.3f} m from the reach pose after 0.5 s")
step(6)
ep, eq = arm_err(pose_q("reach"))
check("arms: reach pose matches build_glb FK", ep < 1e-5 and eq < 1e-8, f"{ep:.2e} m, {eq:.1e}")
H.hotkeys["eddie_grip"](); step(1.0)
ep, eq = arm_err(pose_q("reach", 0.8))
check("arms: grippers close (fingers match FK)", ep < 1e-5 and eq < 1e-8, f"{ep:.2e} m")
H.hotkeys["eddie_grip"](); step(1.0)
H.hotkeys["eddie_arms_wave"](); step(8)
qa = E2.comps["cn:eddie_part_r_grip_base"].p
qa = (qa.x, qa.y, qa.z); step(0.2)
qb = E2.comps["cn:eddie_part_r_grip_base"].p
check("arms: waving keeps the hand moving", math.dist(qa, (qb.x, qb.y, qb.z)) > 0.01)
H.hotkeys["eddie_arms_rest"](); step(8)
ep, eq = arm_err(pose_q("rest"))
check("arms: back to rest exactly", ep < 1e-5 and eq < 1e-8, f"{ep:.2e} m")
check("arms: rest FK matches the .ent placement", rest_place_err() < 1e-5, f"{rest_place_err():.2e}")
n0 = E2.comps["cn:eddie_part_l_arm_shoulder"].n
step(1)
check("arms: idle arms not touched", E2.comps["cn:eddie_part_l_arm_shoulder"].n == n0)

# --- weapons: a copy of V's weapon, held at the tool point, barrel along the fingers
def rz(theta, v):
    c, s_ = math.cos(theta), math.sin(theta)
    return (c * v[0] - s_ * v[1], s_ * v[0] + c * v[1], v[2])


def held_err(side, drop):
    """drop pose vs the gripper's tool point / finger direction, from the components the mod placed"""
    c = E2.comps[f"cn:eddie_part_{side}_grip_base"]
    tool = qrot(c.q, (0, 0, 0.13))
    th = -YAW_SIGN * math.radians(E2.yaw)
    w = rz(th, (c.p.x + tool[0], c.p.y + tool[1], c.p.z + tool[2]))
    want = (E2.pos.x + w[0], E2.pos.y + w[1], E2.pos.z + w[2])
    q = drop.rot._q
    fwd = qrot(q, (0, 1, 0))
    want_fwd = rz(th, qrot(c.q, (0, 0, 1)))
    return math.dist((drop.pos.x, drop.pos.y, drop.pos.z), want), math.dist(fwd, want_fwd), qrot(q, (0, 0, 1))[2]


def reach_q(lg=0.0, rg=0.0):
    return dict(ARMS["vars"], **ARMS["poses"]["reach"], lg=lg, rg=rg)


drops = env.drops
env.weapon.held = False
H.hotkeys["eddie_weapon_right"](); step(0.1)
check("weapons: refused with no weapon drawn", len(drops) == 0)
H.hotkeys["eddie_weapon_fire_test"]()
check("fire test: asks for a weapon when Eddie holds none", len(env.fires) == 0 and STATUS().endswith("give Eddie a weapon first."))
env.weapon.held = True
env.attachDrop(L.table_from({"x": 50.0, "y": 50.0, "z": 0.0}), None)  # someone else's drop, far away
H.hotkeys["eddie_weapon_right"](); step(0.1)
check("weapons: a drop is spawned", len(drops) == 2)
R1 = env.entities[drops[2]]
check("weapons: it's a fresh copy of V's weapon", R1.item.fresh and R1.item.id.tdb == "Items.Preset_Lexington_Default")
check("weapons: V's inventory is unchanged (the copy went straight into the drop)", env.inventory.n == 0)
step(8)
ep, ef, up = held_err("r", R1)
check("weapons: held at the right tool point, barrel along the fingers", ep < 1e-6 and ef < 1e-6,
      f"{ep:.2e} m, fwd off {ef:.2e}")
check("weapons: weapon upright in the Reach pose", up > 0.9, f"up.z={up:.2f}")
ep, _ = arm_err(reach_q(rg=0.45))
check("weapons: arms go to Reach, right gripper closes on it", ep < 1e-5, f"{ep:.2e} m")
H.inputs["eddie_fwd"](True); step(1.0)
ep, ef, _ = held_err("r", R1)
check("weapons: stays in the hand while driving", ep < 1e-6 and ef < 1e-6, f"{ep:.2e} m")
check("weapons: only the drop is moved (the weapon object follows it)", R1.itemObj.rot is None)
H.hotkeys["eddie_weapon_fire_test"]()
f = env.fires[1] if len(env.fires) else None
check("fire test: AIWeapon.Fire with V as owner and the held weapon object",
      f is not None and f.owner.isPlayer and f.weapon.drop.eid == R1.eid and f.t == 12.5 and f.tbh == 1.0,
      "" if f is None else f"isPlayer={f.owner.isPlayer} same={f.weapon.drop.eid == R1.eid} t={f.t} tbh={f.tbh}")
def barrel_cos(drop, pos):
    b = qrot(drop.rot._q, (0, 1, 0))
    d = (pos.x - drop.pos.x, pos.y - drop.pos.y, pos.z - drop.pos.z)
    return sum(x * y for x, y in zip(b, d)) / math.hypot(*d)


check("fire test: the weapon's own trigger mode, fired along the barrel",
      f is not None and f.mode == "FullAuto" and barrel_cos(R1, f.pos) > 1 - 1e-9 and f.target is None)
check("fire test: reports what it found", "IsAttached true" in STATUS() and "AIWeapon.Fire called" in STATUS(),
      STATUS().rsplit(" | ", 1)[-1])
H.inputs["eddie_fwd"](False); step(1.5)

# --- combat: Eddie shoots what V hits
G = L.globals()
npc = G.spawnNPC(E2.pos.x - 8, E2.pos.y + 1, 0.0)
vgun = G.makeGun("vgun")
n0 = len(env.fires)
G.hitBy(npc, vgun); step(3)
chest = (npc.pos.x, npc.pos.y, 1.2)
face = angdiff(E2.yaw, yaw_to(npc.pos.x - E2.pos.x, npc.pos.y - E2.pos.y))
check("combat: Eddie turns to face a character V hit", face < 1.0, f"{face:.2f} deg off")
aimc = barrel_cos(R1, L.table_from({"x": chest[0], "y": chest[1], "z": chest[2]}))
check("combat: the gun points at its chest", aimc > 1 - 1e-9, f"cos {aimc:.9f}")
fl = env.fires[len(env.fires)] if len(env.fires) > n0 else None
check("combat: and fires at it", fl is not None and fl.target is not None and fl.target.eid == npc.eid
      and math.dist((fl.pos.x, fl.pos.y, fl.pos.z), chest) < 1e-9 and fl.owner.isPlayer)
n1 = len(env.fires); step(1.0); rate = len(env.fires) - n1
check("combat: at the weapon's own fire rate", 9 <= rate <= 11, f"{rate} shots/s (weapon: 10)")
npc2 = G.spawnNPC(E2.pos.x - 8, E2.pos.y - 3, 0.0)
G.hitBy(npc2, R1.itemObj); step(0.5)
fl = env.fires[len(env.fires)]
check("combat: Eddie's own hits don't pick a new target", fl.target.eid == npc.eid)
npc.dead = True; step(0.2); n2 = len(env.fires); step(1.0)
check("combat: stops when the target is down", len(env.fires) == n2 and "target down" in STATUS())
ep, ef, _ = held_err("r", R1)
check("combat: the gun swings back into the gripper", ep < 1e-6 and ef < 1e-6, f"fwd off {ef:.2e}")
H.hotkeys["eddie_combat"](); G.hitBy(npc2, vgun); step(1.0)
check("combat: off -> V's hits are ignored", len(env.fires) == n2)
H.hotkeys["eddie_combat"]()

# --- guard: Eddie finds and chases enemies on its own
def last_fire():
    return env.fires[len(env.fires)] if len(env.fires) else None


pl.x, pl.y = E2.pos.x, E2.pos.y + 5
H.hotkeys["eddie_guard"](); step(6)
check("guard: follows V while it's quiet", 1.8 < dist(E2.pos.x, E2.pos.y, pl.x, pl.y) < 2.4,
      f"{dist(E2.pos.x, E2.pos.y, pl.x, pl.y):.2f} m")
calm = G.spawnNPC(E2.pos.x - 6, E2.pos.y + 2, 0.0)
n3 = len(env.fires); step(2)
check("guard: leaves characters alone that aren't fighting V", len(env.fires) == n3)
foe = G.spawnNPC(E2.pos.x - 24, E2.pos.y - 2, 0.0)
foe.fighting = True
step(25)
fl = last_fire()
d_foe = dist(E2.pos.x, E2.pos.y, foe.pos.x, foe.pos.y)
check("guard: picks an enemy fighting V by itself and shoots it", fl is not None and fl.target is not None
      and fl.target.eid == foe.eid and len(env.fires) > n3)
check("guard: closes in to firing distance", 9.0 < d_foe < 11.0, f"{d_foe:.2f} m from the enemy")
face = angdiff(E2.yaw, yaw_to(foe.pos.x - E2.pos.x, foe.pos.y - E2.pos.y))
check("guard: faces it", face < 1.0, f"{face:.2f} deg")
pl.y = pl.y - 40; step(30)
to_v = dist(E2.pos.x, E2.pos.y, pl.x, pl.y)
check("guard: never strays further than the leash from V", to_v < 21.5, f"{to_v:.1f} m from V")
pl.y = pl.y + 40
foe2 = G.spawnNPC(pl.x - 5, pl.y + 3, 0.0)
foe2.fighting = True
foe.dead = True; step(6)
fl = last_fire()
check("guard: moves on to the next enemy when one is down", fl.target.eid == foe2.eid)
H.hotkeys["eddie_stop"](); step(0.5); n4 = len(env.fires); step(1)
check("guard: off -> stops fighting on its own", len(env.fires) == n4)
foe2.dead = True
H.hotkeys["eddie_weapon_left"](); step(2)
L1 = env.entities[drops[3]]
ep, ef, up = held_err("l", L1)
check("weapons: left hand too", len(drops) == 3 and ep < 1e-6 and ef < 1e-6 and up > 0.9, f"{ep:.2e} m up.z={up:.2f}")
z_held = R1.pos.z
H.hotkeys["eddie_grip"](); step(0.15)
check("weapons: released weapons fall", 0.03 < R1.pos.z < z_held - 0.05 and 0.03 < L1.pos.z < z_held,
      f"z {z_held:.2f} -> {R1.pos.z:.2f}")
step(2)
ep, _ = arm_err(reach_q())
check("weapons: opening the grippers lets go", ep < 1e-5, f"{ep:.2e} m")


def lying(d):
    q = d.rot._q
    return abs(qrot(q, (0, 0, 1))[2]) < 1e-6 and abs(qrot(q, (0, 1, 0))[2]) < 1e-6


check("weapons: they land on the floor, lying on their side",
      all(abs(d.pos.z - 0.03) < 1e-9 and lying(d) for d in (R1, L1)), f"z {R1.pos.z:.3f} / {L1.pos.z:.3f}")
n1, n2 = R1.tpn, L1.tpn; step(1)
check("weapons: and stay there", R1.tpn == n1 and L1.tpn == n2)
H.hotkeys["eddie_weapon_right"](); step(2)
env.entities[drops[4]] = None  # V picks it up
step(2)
ep, _ = arm_err(reach_q())
check("weapons: picked up -> gripper opens", ep < 1e-5, f"{ep:.2e} m")
env.weapon.delay = 10 ** 9
H.hotkeys["eddie_weapon_left"](); step(5)
ep, _ = arm_err(reach_q())
check("weapons: gives up if the drop never shows", len(drops) == 4 and ep < 1e-5, f"{ep:.2e} m")
env.weapon.delay = 3
env.setHookOff(True)
H.hotkeys["eddie_weapon_right"](); step(1.0)
S1 = env.entities[drops[len(drops)]]
ep, ef, _ = held_err("r", S1) if S1.rot is not None else (1, 1, 0)
check("weapons: found by scanning when the attach hook doesn't fire", ep < 1e-6 and ef < 1e-6, f"{ep:.2e} m")
env.setHookOff(False)
H.hotkeys["eddie_weapon_drop"](); step(0.5)
env.setLootRefuses(True); n_drops = len(drops)
H.hotkeys["eddie_weapon_right"](); step(4)  # the game's return value can't be trusted: noticed after the wait
check("weapons: a refused drop leaves V no extra copy", env.inventory.n == 0 and len(drops) == n_drops)
env.setLootRefuses(False)
far = env.entities[drops[1]]
check("weapons: other drops nearby are left alone", far.tpn is None)
H.hotkeys["eddie_weapon_right"](); step(1.0)
D = env.entities[drops[len(drops)]]
H.hotkeys["despawn_eddie"](); step(2)
check("weapons: despawning Eddie drops its weapons to the floor", abs(D.pos.z - 0.03) < 1e-9 and lying(D), f"z {D.pos.z:.3f}")
print("ALL PASS" if ok else "SOME FAILED")
sys.exit(0 if ok else 1)
