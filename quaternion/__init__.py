import numpy as np
from typing import Any
from typing_extensions import Self
from math import log,exp,acos,cos,sin


class Quaternion():
    def __init__(self, data: list):
        self.__data = np.array(data, dtype=np.float64)

    def __str__(self) -> str:
        return str(self.__data)
    
    @property
    def data(self) -> np.ndarray:
        return self.__data
    @property
    def real(self) -> float:
        return self.__data[0]
    @property
    def imag(self) -> np.ndarray:
        return self.__data[1:]
    @property
    def norm(self) -> float:
        return np.linalg.norm(self.__data)
    
    def conj(self) -> Self:
        return Quaternion([self.__data[0], -self.__data[1], -self.__data[2], -self.__data[3]])
    
    def inv(self) -> Self:
        return self.conj()
    
    def __neg__(self) -> Self:
        return Quaternion(-self.data)
    
    def __eq__(self, other: Self) -> bool:
        if isinstance(other, Quaternion):
            return (self.data == other.data).all()
        else:
            raise TypeError("unsupported operand type(s) for ==: '{}' and '{}'".format(Quaternion, type(other)) )
    def __ne__(self, other: Self) -> bool:
        return not self.__eq__(other)
    
    def __add__(self, other: Any) -> Self:
        if isinstance(other, Quaternion):
            return Quaternion(self.data + other.data)
        elif isinstance(other, int) or isinstance(other, float):
            return Quaternion([self.data[0]+other, self.data[1], self.data[2], self.data[3]])
        else:
            raise TypeError("unsupported operand type(s) for +: '{}' and '{}'".format(Quaternion, type(other)))
    def __radd__(self, other: Any) -> Self:
        return self.__add__(other)
    def __sub__(self, other: Any) -> Self:
        return self.__add__(-other)
    def __rsub__(self,other: Any) -> Self:
        return -self.__sub__(other)
    
    def __mul__(self, other: Any) -> Self:
        if isinstance(other, Quaternion):
            q1 = self
            q2 = other
            real = q1.real*q2.real - np.dot(q1.imag,q2.imag)
            imag = q1.real*q2.imag + q2.real*q1.imag + np.cross(q1.imag,q2.imag)
            return Quaternion([real, imag[0], imag[1], imag[2]])
        elif isinstance(other, int) or isinstance(other, float):
            # return Quaternion([other*self.data[0], other*self.data[1], other*self.data[2], other*self.data[3]])
            return Quaternion(np.dot(other,self.data))  
        else:
            raise TypeError("unsupported operand type(s) for *: '{}' and '{}'".format(Quaternion, type(other)))
    def __rmul__(self, other: Any) -> Self:
        if isinstance(other, Quaternion):
            q2 = self
            q1 = other
            real = q1.real*q2.real - np.dot(q1.imag,q2.imag)
            imag = q1.real*q2.imag + q2.real*q1.imag + np.cross(q1.imag,q2.imag)
            return Quaternion([real, imag[0], imag[1], imag[2]])
        elif isinstance(other, int) or isinstance(other, float):
            return Quaternion(np.dot(other,self.data))            
        else:
            raise TypeError("unsupported operand type(s) for *: '{}' and '{}'".format(Quaternion, type(other)))
    
    def log(self, always_quaternion: bool = False) -> Self:
        if np.linalg.norm(self.imag) == 0:
            try:
                return Quaternion([log(self.norm),0,0,0])
            except:
                return Quaternion([0,0,0,0])
            # if always_quaternion:
            #     return Quaternion([log(self.norm),0,0,0])
            # else:
            #     return log(self.norm)
        else:
            return log(self.norm) + acos(self.real/self.norm)/np.linalg.norm(self.imag)* Quaternion([0, self.imag[0], self.imag[1], self.imag[2]])
    
    def exp(self, always_quaternion: bool = False) -> Self:
        if np.linalg.norm(self.imag) == 0:
            if always_quaternion:
                return Quaternion([exp(self.real),0,0,0])
            else:
                return exp(self.real)
        else:
            return exp(self.real)*( cos(np.linalg.norm(self.imag)) + sin(np.linalg.norm(self.imag))/np.linalg.norm(self.imag)*Quaternion([0, self.imag[0], self.imag[1], self.imag[2]]) )


class PureQuaternion(Quaternion):
    def __init__(self, data: list):
        super(PureQuaternion,self).__init__([0, data[0], data[1], data[2]])
        self.__pure_data = np.array(data, dtype=np.float64)

    def cross(self,other: Self) -> Self:
        if isinstance(other, PureQuaternion):
            return PureQuaternion(1/2*(self*other - other*self).imag)
        else:
            raise TypeError("unsupported operand type(s) for cross: '{}' and '{}'".format(PureQuaternion, type(other)))

    def dot(self,other: Self) -> float:
        if isinstance(other, PureQuaternion):
            return -1/2*(self*other + other*self).real
        else:
            raise TypeError("unsupported operand type(s) for cross: '{}' and '{}'".format(PureQuaternion, type(other)))


def angular_velocity(q1: Quaternion,q2: Quaternion, dt: float) -> PureQuaternion:
    w_d_data = (2 / dt) * np.array([
        q1.data[0]*q2.data[1] - q1.data[1]*q2.data[0] - q1.data[2]*q2.data[3] + q1.data[3]*q2.data[2],
        q1.data[0]*q2.data[2] + q1.data[1]*q2.data[3] - q1.data[2]*q2.data[0] - q1.data[3]*q2.data[1],
        q1.data[0]*q2.data[3] - q1.data[1]*q2.data[2] + q1.data[2]*q2.data[1] - q1.data[3]*q2.data[0]
        ])
    return PureQuaternion(w_d_data)
