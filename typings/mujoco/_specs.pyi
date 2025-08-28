import flags
import mujoco._enums
import mujoco._structs
import numpy
from typing import Callable, ClassVar, Iterator, overload

class MjByteVec:
    def __init__(self, arg0, arg1: int) -> None: ...
    def __getitem__(self, index): ...
    def __iter__(self): ...
    def __len__(self) -> int: ...
    def __setitem__(self, arg0: int, arg1) -> None: ...

class MjCharVec:
    def __init__(self, arg0: str, arg1: int) -> None: ...
    def __getitem__(self, arg0: int) -> str: ...
    def __iter__(self) -> Iterator[str]: ...
    def __len__(self) -> int: ...
    def __setitem__(self, arg0: int, arg1: str) -> None: ...

class MjOption:
    apirate: float
    ccd_iterations: int
    ccd_tolerance: float
    cone: int
    density: float
    disableactuator: int
    disableflags: int
    enableflags: int
    gravity: numpy.ndarray[numpy.float64[3, 1], flags.writeable]
    impratio: float
    integrator: int
    iterations: int
    jacobian: int
    ls_iterations: int
    ls_tolerance: float
    magnetic: numpy.ndarray[numpy.float64[3, 1], flags.writeable]
    noslip_iterations: int
    noslip_tolerance: float
    o_friction: numpy.ndarray[numpy.float64[5, 1], flags.writeable]
    o_margin: float
    o_solimp: numpy.ndarray[numpy.float64[5, 1], flags.writeable]
    o_solref: numpy.ndarray[numpy.float64[2, 1], flags.writeable]
    sdf_initpoints: int
    sdf_iterations: int
    solver: int
    timestep: float
    tolerance: float
    viscosity: float
    wind: numpy.ndarray[numpy.float64[3, 1], flags.writeable]
    def __init__(self, *args, **kwargs) -> None: ...

class MjSpec:
    from_zip: ClassVar[Callable] = ...
    to_zip: ClassVar[Callable] = ...
    assets: dict
    comment: str
    compiler: MjsCompiler
    copy_during_attach: None
    hasImplicitPluginElem: int
    memory: int
    meshdir: str
    modelfiledir: str
    modelname: str
    nconmax: int
    nemax: int
    njmax: int
    nkey: int
    nstack: int
    nuser_actuator: int
    nuser_body: int
    nuser_cam: int
    nuser_geom: int
    nuser_jnt: int
    nuser_sensor: int
    nuser_site: int
    nuser_tendon: int
    nuserdata: int
    option: MjOption
    override_assets: bool
    stat: MjStatistic
    strippath: int
    texturedir: str
    visual: MjVisual
    def __init__(self) -> None: ...
    def activate_plugin(self, name: str) -> None: ...
    def actuator(self, arg0: str) -> MjsActuator: ...
    def add_actuator(self, default: MjsDefault = ..., **kwargs) -> MjsActuator: ...
    def add_default(self, arg0: str, arg1: MjsDefault) -> MjsDefault: ...
    def add_equality(self, default: MjsDefault = ..., **kwargs) -> MjsEquality: ...
    def add_exclude(self, **kwargs) -> MjsExclude: ...
    def add_flex(self, **kwargs) -> MjsFlex: ...
    def add_hfield(self, **kwargs) -> MjsHField: ...
    def add_key(self, **kwargs) -> MjsKey: ...
    def add_material(self, default: MjsDefault = ..., **kwargs) -> MjsMaterial: ...
    def add_mesh(self, default: MjsDefault = ..., **kwargs) -> MjsMesh: ...
    def add_numeric(self, **kwargs) -> MjsNumeric: ...
    def add_pair(self, default: MjsDefault = ..., **kwargs) -> MjsPair: ...
    def add_plugin(self, **kwargs) -> MjsPlugin: ...
    def add_sensor(self, **kwargs) -> MjsSensor: ...
    def add_skin(self, **kwargs) -> MjsSkin: ...
    def add_tendon(self, default: MjsDefault = ..., **kwargs) -> MjsTendon: ...
    def add_text(self, **kwargs) -> MjsText: ...
    def add_texture(self, **kwargs) -> MjsTexture: ...
    def add_tuple(self, **kwargs) -> MjsTuple: ...
    def attach(self, child: MjSpec, prefix: str | None = ..., suffix: str | None = ..., site: object | None = ..., frame: object | None = ...) -> MjsFrame: ...
    def body(self, arg0: str) -> MjsBody: ...
    def camera(self, arg0: str) -> MjsCamera: ...
    def compile(self) -> object: ...
    def copy(self) -> MjSpec: ...
    def detach_body(self, arg0: MjsBody) -> None: ...
    def detach_default(self, arg0: MjsDefault) -> None: ...
    def equality(self, arg0: str) -> MjsEquality: ...
    def exclude(self, arg0: str) -> MjsExclude: ...
    def find_default(self, arg0: str) -> MjsDefault: ...
    def flex(self, arg0: str) -> MjsFlex: ...
    def frame(self, arg0: str) -> MjsFrame: ...
    @staticmethod
    def from_file(filename: str, include: dict[str, bytes] | None = ..., assets: dict | None = ...) -> MjSpec: ...
    @staticmethod
    def from_string(xml: str, include: dict[str, bytes] | None = ..., assets: dict | None = ...) -> MjSpec: ...
    def geom(self, arg0: str) -> MjsGeom: ...
    def hfield(self, arg0: str) -> MjsHField: ...
    def joint(self, arg0: str) -> MjsJoint: ...
    def key(self, arg0: str) -> MjsKey: ...
    def light(self, arg0: str) -> MjsLight: ...
    def material(self, arg0: str) -> MjsMaterial: ...
    def mesh(self, arg0: str) -> MjsMesh: ...
    def numeric(self, arg0: str) -> MjsNumeric: ...
    def pair(self, arg0: str) -> MjsPair: ...
    def plugin(self, arg0: str) -> MjsPlugin: ...
    def recompile(self, arg0: object, arg1: object) -> object: ...
    def sensor(self, arg0: str) -> MjsSensor: ...
    def site(self, arg0: str) -> MjsSite: ...
    def skin(self, arg0: str) -> MjsSkin: ...
    def tendon(self, arg0: str) -> MjsTendon: ...
    def text(self, arg0: str) -> MjsText: ...
    def texture(self, arg0: str) -> MjsTexture: ...
    def to_file(self, arg0: str) -> None: ...
    def to_xml(self) -> str: ...
    def tuple(self, arg0: str) -> MjsTuple: ...
    @property
    def actuators(self) -> list: ...
    @property
    def bodies(self) -> list: ...
    @property
    def cameras(self) -> list: ...
    @property
    def default(self) -> MjsDefault: ...
    @property
    def equalities(self) -> list: ...
    @property
    def excludes(self) -> list: ...
    @property
    def flexes(self) -> list: ...
    @property
    def frames(self) -> list: ...
    @property
    def geoms(self) -> list: ...
    @property
    def hfields(self) -> list: ...
    @property
    def joints(self) -> list: ...
    @property
    def keys(self) -> list: ...
    @property
    def lights(self) -> list: ...
    @property
    def materials(self) -> list: ...
    @property
    def meshes(self) -> list: ...
    @property
    def numerics(self) -> list: ...
    @property
    def pairs(self) -> list: ...
    @property
    def parent(self) -> MjSpec: ...
    @property
    def plugins(self) -> list: ...
    @property
    def sensors(self) -> list: ...
    @property
    def sites(self) -> list: ...
    @property
    def skins(self) -> list: ...
    @property
    def tendons(self) -> list: ...
    @property
    def texts(self) -> list: ...
    @property
    def textures(self) -> list: ...
    @property
    def tuples(self) -> list: ...
    @property
    def worldbody(self) -> MjsBody: ...

class MjStatistic:
    center: numpy.ndarray[numpy.float64[3, 1], flags.writeable]
    extent: float
    meaninertia: float
    meanmass: float
    meansize: float
    def __init__(self, *args, **kwargs) -> None: ...

class MjStringVec:
    def __init__(self, arg0: str, arg1: int) -> None: ...
    def __getitem__(self, arg0: int) -> str: ...
    def __iter__(self) -> Iterator[str]: ...
    def __len__(self) -> int: ...
    def __setitem__(self, arg0: int, arg1: str) -> None: ...

class MjVisual:
    global_: mujoco._structs.MjVisual.Global
    headlight: MjVisualHeadlight
    map: mujoco._structs.MjVisual.Map
    quality: mujoco._structs.MjVisual.Quality
    rgba: MjVisualRgba
    scale: mujoco._structs.MjVisual.Scale
    def __init__(self, *args, **kwargs) -> None: ...

class MjVisualHeadlight:
    active: int
    ambient: numpy.ndarray[numpy.float32[3, 1], flags.writeable]
    diffuse: numpy.ndarray[numpy.float32[3, 1], flags.writeable]
    specular: numpy.ndarray[numpy.float32[3, 1], flags.writeable]
    def __init__(self, *args, **kwargs) -> None: ...

class MjVisualRgba:
    actuator: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    actuatornegative: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    actuatorpositive: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    bv: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    bvactive: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    camera: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    com: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    connect: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    constraint: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    contactforce: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    contactfriction: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    contactgap: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    contactpoint: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    contacttorque: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    crankbroken: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    fog: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    force: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    frustum: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    haze: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    inertia: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    joint: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    light: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    rangefinder: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    selectpoint: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    slidercrank: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    def __init__(self, *args, **kwargs) -> None: ...

class MjsActuator:
    actdim: int
    actearly: int
    actlimited: int
    actrange: numpy.ndarray[numpy.float64[2, 1], flags.writeable]
    biasprm: numpy.ndarray[numpy.float64[10, 1], flags.writeable]
    biastype: mujoco._enums.mjtBias
    classname: MjsDefault
    cranklength: float
    ctrllimited: int
    ctrlrange: numpy.ndarray[numpy.float64[2, 1], flags.writeable]
    dynprm: numpy.ndarray[numpy.float64[10, 1], flags.writeable]
    dyntype: mujoco._enums.mjtDyn
    forcelimited: int
    forcerange: numpy.ndarray[numpy.float64[2, 1], flags.writeable]
    gainprm: numpy.ndarray[numpy.float64[10, 1], flags.writeable]
    gaintype: mujoco._enums.mjtGain
    gear: numpy.ndarray[numpy.float64[6, 1], flags.writeable]
    group: int
    info: str
    inheritrange: float
    lengthrange: numpy.ndarray[numpy.float64[2, 1], flags.writeable]
    name: str
    plugin: MjsPlugin
    refsite: str
    slidersite: str
    target: str
    trntype: mujoco._enums.mjtTrn
    userdata: numpy.ndarray[numpy.float64]
    def __init__(self, *args, **kwargs) -> None: ...
    def delete(self) -> None: ...
    @property
    def id(self) -> int: ...
    @property
    def signature(self) -> int: ...

class MjsBody:
    alt: MjsOrientation
    childclass: str
    classname: MjsDefault
    explicitinertial: int
    fullinertia: numpy.ndarray[numpy.float64[6, 1], flags.writeable]
    gravcomp: float
    ialt: MjsOrientation
    inertia: numpy.ndarray[numpy.float64[3, 1], flags.writeable]
    info: str
    ipos: numpy.ndarray[numpy.float64[3, 1], flags.writeable]
    iquat: numpy.ndarray[numpy.float64[4, 1], flags.writeable]
    mass: float
    mocap: int
    name: str
    plugin: MjsPlugin
    pos: numpy.ndarray[numpy.float64[3, 1], flags.writeable]
    quat: numpy.ndarray[numpy.float64[4, 1], flags.writeable]
    userdata: numpy.ndarray[numpy.float64]
    def __init__(self, *args, **kwargs) -> None: ...
    def add_body(self, default: MjsDefault = ..., **kwargs) -> MjsBody: ...
    def add_camera(self, default: MjsDefault = ..., **kwargs) -> MjsCamera: ...
    def add_frame(self, default: MjsFrame = ..., **kwargs) -> MjsFrame: ...
    def add_freejoint(self, **kwargs) -> MjsJoint: ...
    def add_geom(self, default: MjsDefault = ..., **kwargs) -> MjsGeom: ...
    def add_joint(self, default: MjsDefault = ..., **kwargs) -> MjsJoint: ...
    def add_light(self, default: MjsDefault = ..., **kwargs) -> MjsLight: ...
    def add_site(self, default: MjsDefault = ..., **kwargs) -> MjsSite: ...
    def attach_frame(self, frame: MjsFrame, prefix: str | None = ..., suffix: str | None = ...) -> MjsFrame: ...
    @overload
    def find_all(self, arg0: mujoco._enums.mjtObj) -> list: ...
    @overload
    def find_all(self, arg0: str) -> list: ...
    def find_child(self, arg0: str) -> MjsBody: ...
    def first_body(self) -> MjsBody: ...
    def first_camera(self) -> MjsCamera: ...
    def first_frame(self) -> MjsFrame: ...
    def first_geom(self) -> MjsGeom: ...
    def first_joint(self) -> MjsJoint: ...
    def first_light(self) -> MjsLight: ...
    def first_site(self) -> MjsSite: ...
    def next_body(self, arg0: MjsBody) -> MjsBody: ...
    def next_camera(self, arg0: MjsCamera) -> MjsCamera: ...
    def next_frame(self, arg0: MjsFrame) -> MjsFrame: ...
    def next_geom(self, arg0: MjsGeom) -> MjsGeom: ...
    def next_joint(self, arg0: MjsJoint) -> MjsJoint: ...
    def next_light(self, arg0: MjsLight) -> MjsLight: ...
    def next_site(self, arg0: MjsSite) -> MjsSite: ...
    def set_frame(self, arg0: MjsFrame) -> None: ...
    def to_frame(self) -> MjsFrame: ...
    @property
    def bodies(self) -> list: ...
    @property
    def cameras(self) -> list: ...
    @property
    def frames(self) -> list: ...
    @property
    def geoms(self) -> list: ...
    @property
    def id(self) -> int: ...
    @property
    def joints(self) -> list: ...
    @property
    def lights(self) -> list: ...
    @property
    def parent(self) -> MjsBody: ...
    @property
    def signature(self) -> int: ...
    @property
    def sites(self) -> list: ...

class MjsCamera:
    alt: MjsOrientation
    classname: MjsDefault
    focal_length: numpy.ndarray[numpy.float32[2, 1], flags.writeable]
    focal_pixel: numpy.ndarray[numpy.float32[2, 1], flags.writeable]
    fovy: float
    info: str
    intrinsic: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    ipd: float
    mode: mujoco._enums.mjtCamLight
    name: str
    orthographic: int
    pos: numpy.ndarray[numpy.float64[3, 1], flags.writeable]
    principal_length: numpy.ndarray[numpy.float32[2, 1], flags.writeable]
    principal_pixel: numpy.ndarray[numpy.float32[2, 1], flags.writeable]
    quat: numpy.ndarray[numpy.float64[4, 1], flags.writeable]
    resolution: numpy.ndarray[numpy.float32[2, 1], flags.writeable]
    sensor_size: numpy.ndarray[numpy.float32[2, 1], flags.writeable]
    targetbody: str
    userdata: numpy.ndarray[numpy.float64]
    def __init__(self, *args, **kwargs) -> None: ...
    def delete(self) -> None: ...
    def set_frame(self, arg0: MjsFrame) -> None: ...
    @property
    def id(self) -> int: ...
    @property
    def parent(self) -> MjsBody: ...
    @property
    def signature(self) -> int: ...

class MjsCompiler:
    LRopt: mujoco._structs.MjLROpt
    alignfree: int
    autolimits: int
    balanceinertia: int
    boundinertia: float
    boundmass: float
    degree: int
    discardvisual: int
    eulerseq: MjCharVec
    fitaabb: int
    fusestatic: int
    inertiafromgeom: int
    inertiagrouprange: numpy.ndarray[numpy.int32[2, 1], flags.writeable]
    saveinertial: int
    settotalmass: float
    usethread: int
    def __init__(self, *args, **kwargs) -> None: ...

class MjsDefault:
    actuator: MjsActuator
    camera: MjsCamera
    equality: MjsEquality
    flex: MjsFlex
    geom: MjsGeom
    joint: MjsJoint
    light: MjsLight
    material: MjsMaterial
    mesh: MjsMesh
    name: str
    pair: MjsPair
    site: MjsSite
    tendon: MjsTendon
    def __init__(self, *args, **kwargs) -> None: ...

class MjsElement:
    def __init__(self, *args, **kwargs) -> None: ...

class MjsEquality:
    active: int
    classname: MjsDefault
    data: numpy.ndarray[numpy.float64[11, 1], flags.writeable]
    info: str
    name: str
    name1: str
    name2: str
    objtype: mujoco._enums.mjtObj
    solimp: numpy.ndarray[numpy.float64[5, 1], flags.writeable]
    solref: numpy.ndarray[numpy.float64[2, 1], flags.writeable]
    type: mujoco._enums.mjtEq
    def __init__(self, *args, **kwargs) -> None: ...
    def delete(self) -> None: ...
    @property
    def id(self) -> int: ...
    @property
    def signature(self) -> int: ...

class MjsExclude:
    bodyname1: str
    bodyname2: str
    info: str
    name: str
    def __init__(self, *args, **kwargs) -> None: ...
    def delete(self) -> None: ...
    @property
    def id(self) -> int: ...
    @property
    def signature(self) -> int: ...

class MjsFlex:
    activelayers: int
    conaffinity: int
    condim: int
    contype: int
    damping: float
    dim: int
    edgedamping: float
    edgestiffness: float
    elem: numpy.ndarray[numpy.int32]
    elemtexcoord: numpy.ndarray[numpy.int32]
    flatskin: int
    friction: numpy.ndarray[numpy.float64[3, 1], flags.writeable]
    gap: float
    group: int
    info: str
    internal: int
    margin: float
    material: str
    name: str
    node: numpy.ndarray[numpy.float64]
    nodebody: MjStringVec
    poisson: float
    priority: int
    radius: float
    rgba: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    selfcollide: int
    solimp: numpy.ndarray[numpy.float64[5, 1], flags.writeable]
    solmix: float
    solref: numpy.ndarray[numpy.float64[2, 1], flags.writeable]
    texcoord: numpy.ndarray[numpy.float32]
    thickness: float
    vert: numpy.ndarray[numpy.float64]
    vertbody: MjStringVec
    young: float
    def __init__(self, *args, **kwargs) -> None: ...
    def delete(self) -> None: ...
    @property
    def id(self) -> int: ...
    @property
    def signature(self) -> int: ...

class MjsFrame:
    alt: MjsOrientation
    childclass: str
    info: str
    name: str
    pos: numpy.ndarray[numpy.float64[3, 1], flags.writeable]
    quat: numpy.ndarray[numpy.float64[4, 1], flags.writeable]
    def __init__(self, *args, **kwargs) -> None: ...
    def attach_body(self, body: MjsBody, prefix: str | None = ..., suffix: str | None = ...) -> MjsBody: ...
    def delete(self) -> None: ...
    def set_frame(self, arg0: MjsFrame) -> None: ...
    @property
    def id(self) -> int: ...
    @property
    def parent(self) -> MjsBody: ...
    @property
    def signature(self) -> int: ...

class MjsGeom:
    alt: MjsOrientation
    classname: MjsDefault
    conaffinity: int
    condim: int
    contype: int
    density: float
    fitscale: float
    fluid_coefs: numpy.ndarray[numpy.float64[5, 1], flags.writeable]
    fluid_ellipsoid: float
    friction: numpy.ndarray[numpy.float64[3, 1], flags.writeable]
    fromto: numpy.ndarray[numpy.float64[6, 1], flags.writeable]
    gap: float
    group: int
    hfieldname: str
    info: str
    margin: float
    mass: float
    material: str
    meshname: str
    name: str
    plugin: MjsPlugin
    pos: numpy.ndarray[numpy.float64[3, 1], flags.writeable]
    priority: int
    quat: numpy.ndarray[numpy.float64[4, 1], flags.writeable]
    rgba: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    size: numpy.ndarray[numpy.float64[3, 1], flags.writeable]
    solimp: numpy.ndarray[numpy.float64[5, 1], flags.writeable]
    solmix: float
    solref: numpy.ndarray[numpy.float64[2, 1], flags.writeable]
    type: mujoco._enums.mjtGeom
    typeinertia: mujoco._enums.mjtGeomInertia
    userdata: numpy.ndarray[numpy.float64]
    def __init__(self, *args, **kwargs) -> None: ...
    def delete(self) -> None: ...
    def set_frame(self, arg0: MjsFrame) -> None: ...
    @property
    def id(self) -> int: ...
    @property
    def parent(self) -> MjsBody: ...
    @property
    def signature(self) -> int: ...

class MjsHField:
    content_type: str
    file: str
    info: str
    name: str
    ncol: int
    nrow: int
    size: numpy.ndarray[numpy.float64[4, 1], flags.writeable]
    userdata: numpy.ndarray[numpy.float32]
    def __init__(self, *args, **kwargs) -> None: ...
    def delete(self) -> None: ...
    @property
    def id(self) -> int: ...
    @property
    def signature(self) -> int: ...

class MjsJoint:
    actfrclimited: int
    actfrcrange: numpy.ndarray[numpy.float64[2, 1], flags.writeable]
    actgravcomp: int
    align: int
    armature: float
    axis: numpy.ndarray[numpy.float64[3, 1], flags.writeable]
    classname: MjsDefault
    damping: float
    frictionloss: float
    group: int
    info: str
    limited: int
    margin: float
    name: str
    pos: numpy.ndarray[numpy.float64[3, 1], flags.writeable]
    range: numpy.ndarray[numpy.float64[2, 1], flags.writeable]
    ref: float
    solimp_friction: numpy.ndarray[numpy.float64[5, 1], flags.writeable]
    solimp_limit: numpy.ndarray[numpy.float64[5, 1], flags.writeable]
    solref_friction: numpy.ndarray[numpy.float64[2, 1], flags.writeable]
    solref_limit: numpy.ndarray[numpy.float64[2, 1], flags.writeable]
    springdamper: numpy.ndarray[numpy.float64[2, 1], flags.writeable]
    springref: float
    stiffness: float
    type: mujoco._enums.mjtJoint
    userdata: numpy.ndarray[numpy.float64]
    def __init__(self, *args, **kwargs) -> None: ...
    def delete(self) -> None: ...
    def set_frame(self, arg0: MjsFrame) -> None: ...
    @property
    def id(self) -> int: ...
    @property
    def parent(self) -> MjsBody: ...
    @property
    def signature(self) -> int: ...

class MjsKey:
    act: numpy.ndarray[numpy.float64]
    ctrl: numpy.ndarray[numpy.float64]
    info: str
    mpos: numpy.ndarray[numpy.float64]
    mquat: numpy.ndarray[numpy.float64]
    name: str
    qpos: numpy.ndarray[numpy.float64]
    qvel: numpy.ndarray[numpy.float64]
    time: float
    def __init__(self, *args, **kwargs) -> None: ...
    def delete(self) -> None: ...
    @property
    def id(self) -> int: ...
    @property
    def signature(self) -> int: ...

class MjsLight:
    active: int
    ambient: numpy.ndarray[numpy.float32[3, 1], flags.writeable]
    attenuation: numpy.ndarray[numpy.float32[3, 1], flags.writeable]
    bulbradius: float
    castshadow: int
    classname: MjsDefault
    cutoff: float
    diffuse: numpy.ndarray[numpy.float32[3, 1], flags.writeable]
    dir: numpy.ndarray[numpy.float64[3, 1], flags.writeable]
    directional: int
    exponent: float
    info: str
    mode: mujoco._enums.mjtCamLight
    name: str
    pos: numpy.ndarray[numpy.float64[3, 1], flags.writeable]
    specular: numpy.ndarray[numpy.float32[3, 1], flags.writeable]
    targetbody: str
    def __init__(self, *args, **kwargs) -> None: ...
    def delete(self) -> None: ...
    def set_frame(self, arg0: MjsFrame) -> None: ...
    @property
    def id(self) -> int: ...
    @property
    def parent(self) -> MjsBody: ...
    @property
    def signature(self) -> int: ...

class MjsMaterial:
    classname: MjsDefault
    emission: float
    info: str
    metallic: float
    name: str
    reflectance: float
    rgba: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    roughness: float
    shininess: float
    specular: float
    texrepeat: numpy.ndarray[numpy.float32[2, 1], flags.writeable]
    textures: MjStringVec
    texuniform: int
    def __init__(self, *args, **kwargs) -> None: ...
    def delete(self) -> None: ...
    @property
    def id(self) -> int: ...
    @property
    def signature(self) -> int: ...

class MjsMesh:
    classname: MjsDefault
    content_type: str
    file: str
    inertia: mujoco._enums.mjtMeshInertia
    info: str
    maxhullvert: int
    name: str
    plugin: MjsPlugin
    refpos: numpy.ndarray[numpy.float64[3, 1], flags.writeable]
    refquat: numpy.ndarray[numpy.float64[4, 1], flags.writeable]
    scale: numpy.ndarray[numpy.float64[3, 1], flags.writeable]
    smoothnormal: int
    userface: numpy.ndarray[numpy.int32]
    userfacetexcoord: numpy.ndarray[numpy.int32]
    usernormal: numpy.ndarray[numpy.float32]
    usertexcoord: numpy.ndarray[numpy.float32]
    uservert: numpy.ndarray[numpy.float32]
    def __init__(self, *args, **kwargs) -> None: ...
    def delete(self) -> None: ...
    @property
    def id(self) -> int: ...
    @property
    def signature(self) -> int: ...

class MjsNumeric:
    data: numpy.ndarray[numpy.float64]
    info: str
    name: str
    size: int
    def __init__(self, *args, **kwargs) -> None: ...
    def delete(self) -> None: ...
    @property
    def id(self) -> int: ...
    @property
    def signature(self) -> int: ...

class MjsOrientation:
    axisangle: numpy.ndarray[numpy.float64[4, 1], flags.writeable]
    euler: numpy.ndarray[numpy.float64[3, 1], flags.writeable]
    type: mujoco._enums.mjtOrientation
    xyaxes: numpy.ndarray[numpy.float64[6, 1], flags.writeable]
    zaxis: numpy.ndarray[numpy.float64[3, 1], flags.writeable]
    def __init__(self, *args, **kwargs) -> None: ...

class MjsPair:
    classname: MjsDefault
    condim: int
    friction: numpy.ndarray[numpy.float64[5, 1], flags.writeable]
    gap: float
    geomname1: str
    geomname2: str
    info: str
    margin: float
    name: str
    solimp: numpy.ndarray[numpy.float64[5, 1], flags.writeable]
    solref: numpy.ndarray[numpy.float64[2, 1], flags.writeable]
    solreffriction: numpy.ndarray[numpy.float64[2, 1], flags.writeable]
    def __init__(self, *args, **kwargs) -> None: ...
    def delete(self) -> None: ...
    @property
    def id(self) -> int: ...
    @property
    def signature(self) -> int: ...

class MjsPlugin:
    active: int
    config: None
    id: int
    info: str
    name: str
    plugin_name: str
    def __init__(self, *args, **kwargs) -> None: ...
    def delete(self) -> None: ...
    @property
    def signature(self) -> int: ...

class MjsSensor:
    cutoff: float
    datatype: mujoco._enums.mjtDataType
    dim: int
    info: str
    name: str
    needstage: mujoco._enums.mjtStage
    noise: float
    objname: str
    objtype: mujoco._enums.mjtObj
    plugin: MjsPlugin
    refname: str
    reftype: mujoco._enums.mjtObj
    type: mujoco._enums.mjtSensor
    userdata: numpy.ndarray[numpy.float64]
    def __init__(self, *args, **kwargs) -> None: ...
    def delete(self) -> None: ...
    @property
    def id(self) -> int: ...
    @property
    def signature(self) -> int: ...

class MjsSite:
    alt: MjsOrientation
    classname: MjsDefault
    fromto: numpy.ndarray[numpy.float64[6, 1], flags.writeable]
    group: int
    info: str
    material: str
    name: str
    pos: numpy.ndarray[numpy.float64[3, 1], flags.writeable]
    quat: numpy.ndarray[numpy.float64[4, 1], flags.writeable]
    rgba: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    size: numpy.ndarray[numpy.float64[3, 1], flags.writeable]
    type: mujoco._enums.mjtGeom
    userdata: numpy.ndarray[numpy.float64]
    def __init__(self, *args, **kwargs) -> None: ...
    def attach_body(self, body: MjsBody, prefix: str | None = ..., suffix: str | None = ...) -> MjsBody: ...
    def delete(self) -> None: ...
    def set_frame(self, arg0: MjsFrame) -> None: ...
    @property
    def id(self) -> int: ...
    @property
    def parent(self) -> MjsBody: ...
    @property
    def signature(self) -> int: ...

class MjsSkin:
    bindpos: numpy.ndarray[numpy.float32]
    bindquat: numpy.ndarray[numpy.float32]
    bodyname: MjStringVec
    face: numpy.ndarray[numpy.int32]
    file: str
    group: int
    inflate: float
    info: str
    material: str
    name: str
    rgba: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    texcoord: numpy.ndarray[numpy.float32]
    vert: numpy.ndarray[numpy.float32]
    vertid: list
    vertweight: list
    def __init__(self, *args, **kwargs) -> None: ...
    def delete(self) -> None: ...
    @property
    def id(self) -> int: ...
    @property
    def signature(self) -> int: ...

class MjsTendon:
    actfrclimited: int
    actfrcrange: numpy.ndarray[numpy.float64[2, 1], flags.writeable]
    armature: float
    damping: float
    frictionloss: float
    group: int
    info: str
    limited: int
    margin: float
    material: str
    name: str
    range: numpy.ndarray[numpy.float64[2, 1], flags.writeable]
    rgba: numpy.ndarray[numpy.float32[4, 1], flags.writeable]
    solimp_friction: numpy.ndarray[numpy.float64[5, 1], flags.writeable]
    solimp_limit: numpy.ndarray[numpy.float64[5, 1], flags.writeable]
    solref_friction: numpy.ndarray[numpy.float64[2, 1], flags.writeable]
    solref_limit: numpy.ndarray[numpy.float64[2, 1], flags.writeable]
    springlength: numpy.ndarray[numpy.float64[2, 1], flags.writeable]
    stiffness: float
    userdata: numpy.ndarray[numpy.float64]
    width: float
    def __init__(self, *args, **kwargs) -> None: ...
    def default(self) -> MjsDefault: ...
    def delete(self) -> None: ...
    def wrap_geom(self, arg0: str, arg1: str) -> MjsWrap: ...
    def wrap_joint(self, arg0: str, arg1: float) -> MjsWrap: ...
    def wrap_pulley(self, arg0: float) -> MjsWrap: ...
    def wrap_site(self, arg0: str) -> MjsWrap: ...
    @property
    def id(self) -> int: ...
    @property
    def signature(self) -> int: ...

class MjsText:
    data: str
    info: str
    name: str
    def __init__(self, *args, **kwargs) -> None: ...
    def delete(self) -> None: ...
    @property
    def id(self) -> int: ...
    @property
    def signature(self) -> int: ...

class MjsTexture:
    builtin: int
    content_type: str
    cubefiles: MjStringVec
    data: MjByteVec
    file: str
    gridlayout: MjCharVec
    gridsize: numpy.ndarray[numpy.int32[2, 1], flags.writeable]
    height: int
    hflip: int
    info: str
    mark: int
    markrgb: numpy.ndarray[numpy.float64[3, 1], flags.writeable]
    name: str
    nchannel: int
    random: float
    rgb1: numpy.ndarray[numpy.float64[3, 1], flags.writeable]
    rgb2: numpy.ndarray[numpy.float64[3, 1], flags.writeable]
    type: mujoco._enums.mjtTexture
    vflip: int
    width: int
    def __init__(self, *args, **kwargs) -> None: ...
    def delete(self) -> None: ...
    @property
    def id(self) -> int: ...
    @property
    def signature(self) -> int: ...

class MjsTuple:
    info: str
    name: str
    objname: MjStringVec
    objprm: numpy.ndarray[numpy.float64]
    objtype: numpy.ndarray[numpy.int32]
    def __init__(self, *args, **kwargs) -> None: ...
    def delete(self) -> None: ...
    @property
    def id(self) -> int: ...
    @property
    def signature(self) -> int: ...

class MjsWrap:
    info: str
    def __init__(self, *args, **kwargs) -> None: ...
