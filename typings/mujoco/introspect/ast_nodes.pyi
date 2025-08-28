import dataclasses
from _typeshed import Incomplete
from typing import Sequence

VALID_TYPE_NAME_PATTERN: Incomplete
C_INVALID_TYPE_NAMES: Incomplete

@dataclasses.dataclass
class ValueType:
    name: str
    is_const: bool = ...
    is_volatile: bool = ...
    def __init__(self, name: str, is_const: bool = False, is_volatile: bool = False) -> None: ...
    def decl(self, name_or_decl: str | None = None) -> str: ...

@dataclasses.dataclass
class ArrayType:
    inner_type: ValueType | PointerType
    extents: tuple[int, ...]
    def __init__(self, inner_type: ValueType | PointerType, extents: Sequence[int]) -> None: ...
    def decl(self, name_or_decl: str | None = None) -> str: ...

@dataclasses.dataclass
class PointerType:
    inner_type: ValueType | ArrayType | PointerType
    is_const: bool = ...
    is_volatile: bool = ...
    is_restrict: bool = ...
    def decl(self, name_or_decl: str | None = None) -> str: ...

@dataclasses.dataclass
class FunctionParameterDecl:
    name: str
    type: ValueType | ArrayType | PointerType
    @property
    def decltype(self) -> str: ...

@dataclasses.dataclass
class FunctionDecl:
    name: str
    return_type: ValueType | ArrayType | PointerType
    parameters: tuple[FunctionParameterDecl, ...]
    doc: str
    def __init__(self, name: str, return_type: ValueType | ArrayType | PointerType, parameters: Sequence[FunctionParameterDecl], doc: str) -> None: ...
    @property
    def decltype(self) -> str: ...

class _EnumDeclValues(dict[str, int]): ...

@dataclasses.dataclass
class EnumDecl:
    name: str
    declname: str
    values: dict[str, int]
    def __init__(self, name: str, declname: str, values: dict[str, int]) -> None: ...

@dataclasses.dataclass
class StructFieldDecl:
    name: str
    type: ValueType | ArrayType | PointerType | AnonymousStructDecl | AnonymousUnionDecl
    doc: str
    array_extent: tuple[str | int, ...] | None = ...
    @property
    def decltype(self) -> str: ...

@dataclasses.dataclass
class AnonymousStructDecl:
    fields: tuple[StructFieldDecl | AnonymousUnionDecl, ...]
    def __init__(self, fields: Sequence[StructFieldDecl]) -> None: ...
    def decl(self, name_or_decl: str | None = None): ...

class AnonymousUnionDecl(AnonymousStructDecl):
    def decl(self, name_or_decl: str | None = None): ...

@dataclasses.dataclass
class StructDecl:
    name: str
    declname: str
    fields: tuple[StructFieldDecl | AnonymousUnionDecl, ...]
    def __init__(self, name: str, declname: str, fields: Sequence[StructFieldDecl | AnonymousUnionDecl]) -> None: ...
    def decl(self, name_or_decl: str | None = None) -> str: ...
