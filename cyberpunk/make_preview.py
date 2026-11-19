"""Write out/preview.html: a three.js viewer with the built GLBs embedded.

Open it in any browser (needs internet for three.js from jsDelivr). Toggle the
collision hulls and the per-part file with the buttons.
"""

import base64
from pathlib import Path

OUT = Path(__file__).resolve().parent / "out"


def b64(name: str) -> str:
    return base64.b64encode((OUT / name).read_bytes()).decode()


HTML = """<!doctype html>
<html><head><meta charset="utf-8"><title>Eddie GLB preview</title>
<style>
 html,body{margin:0;height:100%;background:#16181c;color:#ddd;font:13px system-ui,sans-serif}
 #ui{position:absolute;top:10px;left:10px;display:flex;gap:6px;flex-wrap:wrap}
 button{background:#2a2e35;color:#ddd;border:1px solid #444;border-radius:4px;padding:5px 9px;cursor:pointer}
 button.on{background:#3d6fd6;border-color:#3d6fd6;color:#fff}
 #info{position:absolute;bottom:10px;left:10px;opacity:.8}
</style>
<script type="importmap">{"imports":{"three":"https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js","three/addons/":"https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"}}</script>
</head><body>
<div id="ui">
 <button id="b-static" class="on">static</button><button id="b-parts">parts</button>
 <button id="b-coll">collision</button><button id="b-wire">wireframe</button><button id="b-uv">UV check</button>
</div>
<div id="info"></div>
<script type="module">
import * as THREE from 'three';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {RoomEnvironment} from 'three/addons/environments/RoomEnvironment.js';
const DATA = {static:"__STATIC__", parts:"__PARTS__", coll:"__COLL__"};
const r = new THREE.WebGLRenderer({antialias:true}); r.setPixelRatio(devicePixelRatio);
r.setSize(innerWidth, innerHeight); r.toneMapping = THREE.ACESFilmicToneMapping; document.body.appendChild(r.domElement);
const scene = new THREE.Scene(); scene.background = new THREE.Color(0x16181c);
scene.environment = new THREE.PMREMGenerator(r).fromScene(new RoomEnvironment(), 0.04).texture;
const sun = new THREE.DirectionalLight(0xffffff, 1.5); sun.position.set(2,3,1.5); scene.add(sun);
const cam = new THREE.PerspectiveCamera(40, innerWidth/innerHeight, 0.01, 50); cam.position.set(1.3,1.0,1.6);
const ctl = new OrbitControls(cam, r.domElement); ctl.target.set(0,0.4,0); ctl.update();
scene.add(new THREE.GridHelper(4, 40, 0x555555, 0x2c2c2c));
const axes = new THREE.AxesHelper(0.3); scene.add(axes); // red X, green Y(up), blue Z
const front = new THREE.ArrowHelper(new THREE.Vector3(0,0,-1), new THREE.Vector3(0,0.02,0), 0.6, 0xffaa00); scene.add(front);
const loader = new GLTFLoader(), objs = {};
const buf = s => Uint8Array.from(atob(s), c => c.charCodeAt(0)).buffer;
const uvTex = (()=>{const c=document.createElement('canvas');c.width=c.height=256;const g=c.getContext('2d');
 for(let i=0;i<8;i++)for(let j=0;j<8;j++){g.fillStyle=(i+j)%2?'#e33':'#eee';g.fillRect(i*32,j*32,32,32);}
 const t=new THREE.CanvasTexture(c);t.wrapS=t.wrapT=THREE.RepeatWrapping;t.colorSpace=THREE.SRGBColorSpace;return t;})();
let tris = 0;
for (const k of Object.keys(DATA)) {
  const g = await loader.parseAsync(buf(DATA[k]), '');
  objs[k] = g.scene; scene.add(g.scene); g.scene.visible = k === 'static';
  g.scene.traverse(o => { if (!o.isMesh) return;
    if (k === 'coll') o.material = new THREE.MeshBasicMaterial({color:0x33ff88, wireframe:true, transparent:true, opacity:.6});
    else { o.userData.mat = o.material; if (k==='static') tris += o.geometry.index.count/3; }
  });
}
document.getElementById('info').textContent = `static: ${tris.toLocaleString()} triangles · orange arrow = robot front (-Z in glTF = +Y after import)`;
const tog = (id, fn) => { const b = document.getElementById(id); b.onclick = () => { b.classList.toggle('on'); fn(b.classList.contains('on')); }; };
tog('b-static', on => objs.static.visible = on); tog('b-parts', on => objs.parts.visible = on);
tog('b-coll', on => objs.coll.visible = on);
tog('b-wire', on => ['static','parts'].forEach(k => objs[k].traverse(o => { if (o.isMesh) o.userData.mat.wireframe = on; })));
tog('b-uv', on => ['static','parts'].forEach(k => objs[k].traverse(o => { if (o.isMesh) o.material = on ?
  new THREE.MeshStandardMaterial({map: uvTex}) : o.userData.mat; })));
addEventListener('resize', () => { cam.aspect = innerWidth/innerHeight; cam.updateProjectionMatrix(); r.setSize(innerWidth, innerHeight); });
r.setAnimationLoop(() => r.render(scene, cam));
window.__ready = true; window.view = (p, t) => { cam.position.set(...p); ctl.target.set(...t); ctl.update(); };
</script></body></html>
"""

if __name__ == "__main__":
    html = (HTML.replace("__STATIC__", b64("eddie_static.glb"))
                .replace("__PARTS__", b64("eddie_parts.glb"))
                .replace("__COLL__", b64("eddie_collision.glb")))
    (OUT / "preview.html").write_text(html, encoding="utf-8")
    print(OUT / "preview.html")
