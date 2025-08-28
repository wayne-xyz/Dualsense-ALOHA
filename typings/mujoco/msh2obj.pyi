import dataclasses
import numpy as np
import pathlib

@dataclasses.dataclass(frozen=True)
class Msh:
    vertex_positions: np.ndarray
    vertex_normals: np.ndarray
    vertex_texcoords: np.ndarray
    face_vertex_indices: np.ndarray
    @staticmethod
    def create(file: pathlib.Path) -> Msh: ...

def msh_to_obj(msh_file: pathlib.Path) -> str: ...
