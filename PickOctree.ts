import { Vector3, Sphere, Raycaster, Box3, Intersection } from 'three';
import { isCombineMesh, isInstancedPrimitive, isPrimitive, isVector3 } from './types';

import { InstancedPrimitive } from '../engine/InstancedPrimitive';
import { Primitive } from '../engine/Primitive';
import { WorkerPool } from '../libs/WorkerPool';
import { CombineMesh } from '../engine/CombineMesh';

export interface IOctree {
    position: Vector3 | number[];
    radius: number;
    objectsThreshold?: number;
    overlapPct?: number; //子节点重叠率
    depthMax?: number;
}

export class PickOctree {
    pool: WorkerPool = new WorkerPool();
    isReady = false;
    private _primitiveCache: { [key: string]: CombineMesh | Primitive | InstancedPrimitive } = {};

    constructor() {
        const workerContent = `
 

// #region THREE
class Quaternion {

    constructor(x = 0, y = 0, z = 0, w = 1) {

        this._x = x;
        this._y = y;
        this._z = z;
        this._w = w;

    }

    static slerp(qa, qb, qm, t) {

        console.warn('THREE.Quaternion: Static .slerp() has been deprecated. Use qm.slerpQuaternions( qa, qb, t ) instead.');
        return qm.slerpQuaternions(qa, qb, t);

    }

    static slerpFlat(dst, dstOffset, src0, srcOffset0, src1, srcOffset1, t) {

        // fuzz-free, array-based Quaternion SLERP operation

        let x0 = src0[srcOffset0 + 0],
            y0 = src0[srcOffset0 + 1],
            z0 = src0[srcOffset0 + 2],
            w0 = src0[srcOffset0 + 3];

        const x1 = src1[srcOffset1 + 0],
            y1 = src1[srcOffset1 + 1],
            z1 = src1[srcOffset1 + 2],
            w1 = src1[srcOffset1 + 3];

        if (t === 0) {

            dst[dstOffset + 0] = x0;
            dst[dstOffset + 1] = y0;
            dst[dstOffset + 2] = z0;
            dst[dstOffset + 3] = w0;
            return;

        }

        if (t === 1) {

            dst[dstOffset + 0] = x1;
            dst[dstOffset + 1] = y1;
            dst[dstOffset + 2] = z1;
            dst[dstOffset + 3] = w1;
            return;

        }

        if (w0 !== w1 || x0 !== x1 || y0 !== y1 || z0 !== z1) {

            let s = 1 - t;
            const cos = x0 * x1 + y0 * y1 + z0 * z1 + w0 * w1,
                dir = (cos >= 0 ? 1 : - 1),
                sqrSin = 1 - cos * cos;

            // Skip the Slerp for tiny steps to avoid numeric problems:
            if (sqrSin > Number.EPSILON) {

                const sin = Math.sqrt(sqrSin),
                    len = Math.atan2(sin, cos * dir);

                s = Math.sin(s * len) / sin;
                t = Math.sin(t * len) / sin;

            }

            const tDir = t * dir;

            x0 = x0 * s + x1 * tDir;
            y0 = y0 * s + y1 * tDir;
            z0 = z0 * s + z1 * tDir;
            w0 = w0 * s + w1 * tDir;

            // Normalize in case we just did a lerp:
            if (s === 1 - t) {

                const f = 1 / Math.sqrt(x0 * x0 + y0 * y0 + z0 * z0 + w0 * w0);

                x0 *= f;
                y0 *= f;
                z0 *= f;
                w0 *= f;

            }

        }

        dst[dstOffset] = x0;
        dst[dstOffset + 1] = y0;
        dst[dstOffset + 2] = z0;
        dst[dstOffset + 3] = w0;

    }

    static multiplyQuaternionsFlat(dst, dstOffset, src0, srcOffset0, src1, srcOffset1) {

        const x0 = src0[srcOffset0];
        const y0 = src0[srcOffset0 + 1];
        const z0 = src0[srcOffset0 + 2];
        const w0 = src0[srcOffset0 + 3];

        const x1 = src1[srcOffset1];
        const y1 = src1[srcOffset1 + 1];
        const z1 = src1[srcOffset1 + 2];
        const w1 = src1[srcOffset1 + 3];

        dst[dstOffset] = x0 * w1 + w0 * x1 + y0 * z1 - z0 * y1;
        dst[dstOffset + 1] = y0 * w1 + w0 * y1 + z0 * x1 - x0 * z1;
        dst[dstOffset + 2] = z0 * w1 + w0 * z1 + x0 * y1 - y0 * x1;
        dst[dstOffset + 3] = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1;

        return dst;

    }

    get x() {

        return this._x;

    }

    set x(value) {

        this._x = value;
        this._onChangeCallback();

    }

    get y() {

        return this._y;

    }

    set y(value) {

        this._y = value;
        this._onChangeCallback();

    }

    get z() {

        return this._z;

    }

    set z(value) {

        this._z = value;
        this._onChangeCallback();

    }

    get w() {

        return this._w;

    }

    set w(value) {

        this._w = value;
        this._onChangeCallback();

    }

    set(x, y, z, w) {

        this._x = x;
        this._y = y;
        this._z = z;
        this._w = w;

        this._onChangeCallback();

        return this;

    }

    clone() {

        return new this.constructor(this._x, this._y, this._z, this._w);

    }

    copy(quaternion) {

        this._x = quaternion.x;
        this._y = quaternion.y;
        this._z = quaternion.z;
        this._w = quaternion.w;

        this._onChangeCallback();

        return this;

    }

    setFromEuler(euler, update) {

        if (!(euler && euler.isEuler)) {

            throw new Error('THREE.Quaternion: .setFromEuler() now expects an Euler rotation rather than a Vector3 and order.');

        }

        const x = euler._x, y = euler._y, z = euler._z, order = euler._order;

        // http://www.mathworks.com/matlabcentral/fileexchange/
        // 	20696-function-to-convert-between-dcm-euler-angles-quaternions-and-euler-vectors/
        //	content/SpinCalc.m

        const cos = Math.cos;
        const sin = Math.sin;

        const c1 = cos(x / 2);
        const c2 = cos(y / 2);
        const c3 = cos(z / 2);

        const s1 = sin(x / 2);
        const s2 = sin(y / 2);
        const s3 = sin(z / 2);

        switch (order) {

            case 'XYZ':
                this._x = s1 * c2 * c3 + c1 * s2 * s3;
                this._y = c1 * s2 * c3 - s1 * c2 * s3;
                this._z = c1 * c2 * s3 + s1 * s2 * c3;
                this._w = c1 * c2 * c3 - s1 * s2 * s3;
                break;

            case 'YXZ':
                this._x = s1 * c2 * c3 + c1 * s2 * s3;
                this._y = c1 * s2 * c3 - s1 * c2 * s3;
                this._z = c1 * c2 * s3 - s1 * s2 * c3;
                this._w = c1 * c2 * c3 + s1 * s2 * s3;
                break;

            case 'ZXY':
                this._x = s1 * c2 * c3 - c1 * s2 * s3;
                this._y = c1 * s2 * c3 + s1 * c2 * s3;
                this._z = c1 * c2 * s3 + s1 * s2 * c3;
                this._w = c1 * c2 * c3 - s1 * s2 * s3;
                break;

            case 'ZYX':
                this._x = s1 * c2 * c3 - c1 * s2 * s3;
                this._y = c1 * s2 * c3 + s1 * c2 * s3;
                this._z = c1 * c2 * s3 - s1 * s2 * c3;
                this._w = c1 * c2 * c3 + s1 * s2 * s3;
                break;

            case 'YZX':
                this._x = s1 * c2 * c3 + c1 * s2 * s3;
                this._y = c1 * s2 * c3 + s1 * c2 * s3;
                this._z = c1 * c2 * s3 - s1 * s2 * c3;
                this._w = c1 * c2 * c3 - s1 * s2 * s3;
                break;

            case 'XZY':
                this._x = s1 * c2 * c3 - c1 * s2 * s3;
                this._y = c1 * s2 * c3 - s1 * c2 * s3;
                this._z = c1 * c2 * s3 + s1 * s2 * c3;
                this._w = c1 * c2 * c3 + s1 * s2 * s3;
                break;

            default:
                console.warn('THREE.Quaternion: .setFromEuler() encountered an unknown order: ' + order);

        }

        if (update !== false) this._onChangeCallback();

        return this;

    }

    setFromAxisAngle(axis, angle) {

        // http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/index.htm

        // assumes axis is normalized

        const halfAngle = angle / 2, s = Math.sin(halfAngle);

        this._x = axis.x * s;
        this._y = axis.y * s;
        this._z = axis.z * s;
        this._w = Math.cos(halfAngle);

        this._onChangeCallback();

        return this;

    }

    setFromRotationMatrix(m) {

        // http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm

        // assumes the upper 3x3 of m is a pure rotation matrix (i.e, unscaled)

        const te = m.elements,

            m11 = te[0], m12 = te[4], m13 = te[8],
            m21 = te[1], m22 = te[5], m23 = te[9],
            m31 = te[2], m32 = te[6], m33 = te[10],

            trace = m11 + m22 + m33;

        if (trace > 0) {

            const s = 0.5 / Math.sqrt(trace + 1.0);

            this._w = 0.25 / s;
            this._x = (m32 - m23) * s;
            this._y = (m13 - m31) * s;
            this._z = (m21 - m12) * s;

        } else if (m11 > m22 && m11 > m33) {

            const s = 2.0 * Math.sqrt(1.0 + m11 - m22 - m33);

            this._w = (m32 - m23) / s;
            this._x = 0.25 * s;
            this._y = (m12 + m21) / s;
            this._z = (m13 + m31) / s;

        } else if (m22 > m33) {

            const s = 2.0 * Math.sqrt(1.0 + m22 - m11 - m33);

            this._w = (m13 - m31) / s;
            this._x = (m12 + m21) / s;
            this._y = 0.25 * s;
            this._z = (m23 + m32) / s;

        } else {

            const s = 2.0 * Math.sqrt(1.0 + m33 - m11 - m22);

            this._w = (m21 - m12) / s;
            this._x = (m13 + m31) / s;
            this._y = (m23 + m32) / s;
            this._z = 0.25 * s;

        }

        this._onChangeCallback();

        return this;

    }

    setFromUnitVectors(vFrom, vTo) {

        // assumes direction vectors vFrom and vTo are normalized

        let r = vFrom.dot(vTo) + 1;

        if (r < Number.EPSILON) {

            // vFrom and vTo point in opposite directions

            r = 0;

            if (Math.abs(vFrom.x) > Math.abs(vFrom.z)) {

                this._x = - vFrom.y;
                this._y = vFrom.x;
                this._z = 0;
                this._w = r;

            } else {

                this._x = 0;
                this._y = - vFrom.z;
                this._z = vFrom.y;
                this._w = r;

            }

        } else {

            // crossVectors( vFrom, vTo ); // inlined to avoid cyclic dependency on Vector3

            this._x = vFrom.y * vTo.z - vFrom.z * vTo.y;
            this._y = vFrom.z * vTo.x - vFrom.x * vTo.z;
            this._z = vFrom.x * vTo.y - vFrom.y * vTo.x;
            this._w = r;

        }

        return this.normalize();

    }

    angleTo(q) {

        return 2 * Math.acos(Math.abs(clamp(this.dot(q), - 1, 1)));

    }

    rotateTowards(q, step) {

        const angle = this.angleTo(q);

        if (angle === 0) return this;

        const t = Math.min(1, step / angle);

        this.slerp(q, t);

        return this;

    }

    identity() {

        return this.set(0, 0, 0, 1);

    }

    invert() {

        // quaternion is assumed to have unit length

        return this.conjugate();

    }

    conjugate() {

        this._x *= - 1;
        this._y *= - 1;
        this._z *= - 1;

        this._onChangeCallback();

        return this;

    }

    dot(v) {

        return this._x * v._x + this._y * v._y + this._z * v._z + this._w * v._w;

    }

    lengthSq() {

        return this._x * this._x + this._y * this._y + this._z * this._z + this._w * this._w;

    }

    length() {

        return Math.sqrt(this._x * this._x + this._y * this._y + this._z * this._z + this._w * this._w);

    }

    normalize() {

        let l = this.length();

        if (l === 0) {

            this._x = 0;
            this._y = 0;
            this._z = 0;
            this._w = 1;

        } else {

            l = 1 / l;

            this._x = this._x * l;
            this._y = this._y * l;
            this._z = this._z * l;
            this._w = this._w * l;

        }

        this._onChangeCallback();

        return this;

    }

    multiply(q, p) {

        if (p !== undefined) {

            console.warn('THREE.Quaternion: .multiply() now only accepts one argument. Use .multiplyQuaternions( a, b ) instead.');
            return this.multiplyQuaternions(q, p);

        }

        return this.multiplyQuaternions(this, q);

    }

    premultiply(q) {

        return this.multiplyQuaternions(q, this);

    }

    multiplyQuaternions(a, b) {

        // from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/code/index.htm

        const qax = a._x, qay = a._y, qaz = a._z, qaw = a._w;
        const qbx = b._x, qby = b._y, qbz = b._z, qbw = b._w;

        this._x = qax * qbw + qaw * qbx + qay * qbz - qaz * qby;
        this._y = qay * qbw + qaw * qby + qaz * qbx - qax * qbz;
        this._z = qaz * qbw + qaw * qbz + qax * qby - qay * qbx;
        this._w = qaw * qbw - qax * qbx - qay * qby - qaz * qbz;

        this._onChangeCallback();

        return this;

    }

    slerp(qb, t) {

        if (t === 0) return this;
        if (t === 1) return this.copy(qb);

        const x = this._x, y = this._y, z = this._z, w = this._w;

        // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/

        let cosHalfTheta = w * qb._w + x * qb._x + y * qb._y + z * qb._z;

        if (cosHalfTheta < 0) {

            this._w = - qb._w;
            this._x = - qb._x;
            this._y = - qb._y;
            this._z = - qb._z;

            cosHalfTheta = - cosHalfTheta;

        } else {

            this.copy(qb);

        }

        if (cosHalfTheta >= 1.0) {

            this._w = w;
            this._x = x;
            this._y = y;
            this._z = z;

            return this;

        }

        const sqrSinHalfTheta = 1.0 - cosHalfTheta * cosHalfTheta;

        if (sqrSinHalfTheta <= Number.EPSILON) {

            const s = 1 - t;
            this._w = s * w + t * this._w;
            this._x = s * x + t * this._x;
            this._y = s * y + t * this._y;
            this._z = s * z + t * this._z;

            this.normalize();
            this._onChangeCallback();

            return this;

        }

        const sinHalfTheta = Math.sqrt(sqrSinHalfTheta);
        const halfTheta = Math.atan2(sinHalfTheta, cosHalfTheta);
        const ratioA = Math.sin((1 - t) * halfTheta) / sinHalfTheta,
            ratioB = Math.sin(t * halfTheta) / sinHalfTheta;

        this._w = (w * ratioA + this._w * ratioB);
        this._x = (x * ratioA + this._x * ratioB);
        this._y = (y * ratioA + this._y * ratioB);
        this._z = (z * ratioA + this._z * ratioB);

        this._onChangeCallback();

        return this;

    }

    slerpQuaternions(qa, qb, t) {

        this.copy(qa).slerp(qb, t);

    }

    equals(quaternion) {

        return (quaternion._x === this._x) && (quaternion._y === this._y) && (quaternion._z === this._z) && (quaternion._w === this._w);

    }

    fromArray(array, offset = 0) {

        this._x = array[offset];
        this._y = array[offset + 1];
        this._z = array[offset + 2];
        this._w = array[offset + 3];

        this._onChangeCallback();

        return this;

    }

    toArray(array = [], offset = 0) {

        array[offset] = this._x;
        array[offset + 1] = this._y;
        array[offset + 2] = this._z;
        array[offset + 3] = this._w;

        return array;

    }

    fromBufferAttribute(attribute, index) {

        this._x = attribute.getX(index);
        this._y = attribute.getY(index);
        this._z = attribute.getZ(index);
        this._w = attribute.getW(index);

        return this;

    }

    _onChange(callback) {

        this._onChangeCallback = callback;

        return this;

    }

    _onChangeCallback() { }

}

Quaternion.prototype.isQuaternion = true;

class Vector3 {

    constructor(x = 0, y = 0, z = 0) {

        this.x = x;
        this.y = y;
        this.z = z;

    }

    set(x, y, z) {

        if (z === undefined) z = this.z; // sprite.scale.set(x,y)

        this.x = x;
        this.y = y;
        this.z = z;

        return this;

    }

    setScalar(scalar) {

        this.x = scalar;
        this.y = scalar;
        this.z = scalar;

        return this;

    }

    setX(x) {

        this.x = x;

        return this;

    }

    setY(y) {

        this.y = y;

        return this;

    }

    setZ(z) {

        this.z = z;

        return this;

    }

    setComponent(index, value) {

        switch (index) {

            case 0: this.x = value; break;
            case 1: this.y = value; break;
            case 2: this.z = value; break;
            default: throw new Error('index is out of range: ' + index);

        }

        return this;

    }

    getComponent(index) {

        switch (index) {

            case 0: return this.x;
            case 1: return this.y;
            case 2: return this.z;
            default: throw new Error('index is out of range: ' + index);

        }

    }

    clone() {

        return new this.constructor(this.x, this.y, this.z);

    }

    copy(v) {

        this.x = v.x;
        this.y = v.y;
        this.z = v.z;

        return this;

    }

    add(v, w) {

        if (w !== undefined) {

            console.warn('THREE.Vector3: .add() now only accepts one argument. Use .addVectors( a, b ) instead.');
            return this.addVectors(v, w);

        }

        this.x += v.x;
        this.y += v.y;
        this.z += v.z;

        return this;

    }

    addScalar(s) {

        this.x += s;
        this.y += s;
        this.z += s;

        return this;

    }

    addVectors(a, b) {

        this.x = a.x + b.x;
        this.y = a.y + b.y;
        this.z = a.z + b.z;

        return this;

    }

    addScaledVector(v, s) {

        this.x += v.x * s;
        this.y += v.y * s;
        this.z += v.z * s;

        return this;

    }

    sub(v, w) {

        if (w !== undefined) {

            console.warn('THREE.Vector3: .sub() now only accepts one argument. Use .subVectors( a, b ) instead.');
            return this.subVectors(v, w);

        }

        this.x -= v.x;
        this.y -= v.y;
        this.z -= v.z;

        return this;

    }

    subScalar(s) {

        this.x -= s;
        this.y -= s;
        this.z -= s;

        return this;

    }

    subVectors(a, b) {

        this.x = a.x - b.x;
        this.y = a.y - b.y;
        this.z = a.z - b.z;

        return this;

    }

    multiply(v, w) {

        if (w !== undefined) {

            console.warn('THREE.Vector3: .multiply() now only accepts one argument. Use .multiplyVectors( a, b ) instead.');
            return this.multiplyVectors(v, w);

        }

        this.x *= v.x;
        this.y *= v.y;
        this.z *= v.z;

        return this;

    }

    multiplyScalar(scalar) {

        this.x *= scalar;
        this.y *= scalar;
        this.z *= scalar;

        return this;

    }

    multiplyVectors(a, b) {

        this.x = a.x * b.x;
        this.y = a.y * b.y;
        this.z = a.z * b.z;

        return this;

    }

    applyEuler(euler) {

        if (!(euler && euler.isEuler)) {

            console.error('THREE.Vector3: .applyEuler() now expects an Euler rotation rather than a Vector3 and order.');

        }

        return this.applyQuaternion(_quaternion$4.setFromEuler(euler));

    }

    applyAxisAngle(axis, angle) {

        return this.applyQuaternion(_quaternion$4.setFromAxisAngle(axis, angle));

    }

    applyMatrix3(m) {

        const x = this.x, y = this.y, z = this.z;
        const e = m.elements;

        this.x = e[0] * x + e[3] * y + e[6] * z;
        this.y = e[1] * x + e[4] * y + e[7] * z;
        this.z = e[2] * x + e[5] * y + e[8] * z;

        return this;

    }

    applyNormalMatrix(m) {

        return this.applyMatrix3(m).normalize();

    }

    applyMatrix4(m) {

        const x = this.x, y = this.y, z = this.z;
        const e = m.elements;

        const w = 1 / (e[3] * x + e[7] * y + e[11] * z + e[15]);

        this.x = (e[0] * x + e[4] * y + e[8] * z + e[12]) * w;
        this.y = (e[1] * x + e[5] * y + e[9] * z + e[13]) * w;
        this.z = (e[2] * x + e[6] * y + e[10] * z + e[14]) * w;

        return this;

    }

    applyQuaternion(q) {

        const x = this.x, y = this.y, z = this.z;
        const qx = q.x, qy = q.y, qz = q.z, qw = q.w;

        // calculate quat * vector

        const ix = qw * x + qy * z - qz * y;
        const iy = qw * y + qz * x - qx * z;
        const iz = qw * z + qx * y - qy * x;
        const iw = - qx * x - qy * y - qz * z;

        // calculate result * inverse quat

        this.x = ix * qw + iw * - qx + iy * - qz - iz * - qy;
        this.y = iy * qw + iw * - qy + iz * - qx - ix * - qz;
        this.z = iz * qw + iw * - qz + ix * - qy - iy * - qx;

        return this;

    }

    project(camera) {

        return this.applyMatrix4(camera.matrixWorldInverse).applyMatrix4(camera.projectionMatrix);

    }

    unproject(camera) {

        return this.applyMatrix4(camera.projectionMatrixInverse).applyMatrix4(camera.matrixWorld);

    }

    transformDirection(m) {

        // input: THREE.Matrix4 affine matrix
        // vector interpreted as a direction

        const x = this.x, y = this.y, z = this.z;
        const e = m.elements;

        this.x = e[0] * x + e[4] * y + e[8] * z;
        this.y = e[1] * x + e[5] * y + e[9] * z;
        this.z = e[2] * x + e[6] * y + e[10] * z;

        return this.normalize();

    }

    divide(v) {

        this.x /= v.x;
        this.y /= v.y;
        this.z /= v.z;

        return this;

    }

    divideScalar(scalar) {

        return this.multiplyScalar(1 / scalar);

    }

    min(v) {

        this.x = Math.min(this.x, v.x);
        this.y = Math.min(this.y, v.y);
        this.z = Math.min(this.z, v.z);

        return this;

    }

    max(v) {

        this.x = Math.max(this.x, v.x);
        this.y = Math.max(this.y, v.y);
        this.z = Math.max(this.z, v.z);

        return this;

    }

    clamp(min, max) {

        // assumes min < max, componentwise

        this.x = Math.max(min.x, Math.min(max.x, this.x));
        this.y = Math.max(min.y, Math.min(max.y, this.y));
        this.z = Math.max(min.z, Math.min(max.z, this.z));

        return this;

    }

    clampScalar(minVal, maxVal) {

        this.x = Math.max(minVal, Math.min(maxVal, this.x));
        this.y = Math.max(minVal, Math.min(maxVal, this.y));
        this.z = Math.max(minVal, Math.min(maxVal, this.z));

        return this;

    }

    clampLength(min, max) {

        const length = this.length();

        return this.divideScalar(length || 1).multiplyScalar(Math.max(min, Math.min(max, length)));

    }

    floor() {

        this.x = Math.floor(this.x);
        this.y = Math.floor(this.y);
        this.z = Math.floor(this.z);

        return this;

    }

    ceil() {

        this.x = Math.ceil(this.x);
        this.y = Math.ceil(this.y);
        this.z = Math.ceil(this.z);

        return this;

    }

    round() {

        this.x = Math.round(this.x);
        this.y = Math.round(this.y);
        this.z = Math.round(this.z);

        return this;

    }

    roundToZero() {

        this.x = (this.x < 0) ? Math.ceil(this.x) : Math.floor(this.x);
        this.y = (this.y < 0) ? Math.ceil(this.y) : Math.floor(this.y);
        this.z = (this.z < 0) ? Math.ceil(this.z) : Math.floor(this.z);

        return this;

    }

    negate() {

        this.x = - this.x;
        this.y = - this.y;
        this.z = - this.z;

        return this;

    }

    dot(v) {

        return this.x * v.x + this.y * v.y + this.z * v.z;

    }

    // TODO lengthSquared?

    lengthSq() {

        return this.x * this.x + this.y * this.y + this.z * this.z;

    }

    length() {

        return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);

    }

    manhattanLength() {

        return Math.abs(this.x) + Math.abs(this.y) + Math.abs(this.z);

    }

    normalize() {

        return this.divideScalar(this.length() || 1);

    }

    setLength(length) {

        return this.normalize().multiplyScalar(length);

    }

    lerp(v, alpha) {

        this.x += (v.x - this.x) * alpha;
        this.y += (v.y - this.y) * alpha;
        this.z += (v.z - this.z) * alpha;

        return this;

    }

    lerpVectors(v1, v2, alpha) {

        this.x = v1.x + (v2.x - v1.x) * alpha;
        this.y = v1.y + (v2.y - v1.y) * alpha;
        this.z = v1.z + (v2.z - v1.z) * alpha;

        return this;

    }

    cross(v, w) {

        if (w !== undefined) {

            console.warn('THREE.Vector3: .cross() now only accepts one argument. Use .crossVectors( a, b ) instead.');
            return this.crossVectors(v, w);

        }

        return this.crossVectors(this, v);

    }

    crossVectors(a, b) {

        const ax = a.x, ay = a.y, az = a.z;
        const bx = b.x, by = b.y, bz = b.z;

        this.x = ay * bz - az * by;
        this.y = az * bx - ax * bz;
        this.z = ax * by - ay * bx;

        return this;

    }

    projectOnVector(v) {

        const denominator = v.lengthSq();

        if (denominator === 0) return this.set(0, 0, 0);

        const scalar = v.dot(this) / denominator;

        return this.copy(v).multiplyScalar(scalar);

    }

    projectOnPlane(planeNormal) {

        _vector$c.copy(this).projectOnVector(planeNormal);

        return this.sub(_vector$c);

    }

    reflect(normal) {

        // reflect incident vector off plane orthogonal to normal
        // normal is assumed to have unit length

        return this.sub(_vector$c.copy(normal).multiplyScalar(2 * this.dot(normal)));

    }

    angleTo(v) {

        const denominator = Math.sqrt(this.lengthSq() * v.lengthSq());

        if (denominator === 0) return Math.PI / 2;

        const theta = this.dot(v) / denominator;

        // clamp, to handle numerical problems

        return Math.acos(clamp(theta, - 1, 1));

    }

    distanceTo(v) {

        return Math.sqrt(this.distanceToSquared(v));

    }

    distanceToSquared(v) {

        const dx = this.x - v.x, dy = this.y - v.y, dz = this.z - v.z;

        return dx * dx + dy * dy + dz * dz;

    }

    manhattanDistanceTo(v) {

        return Math.abs(this.x - v.x) + Math.abs(this.y - v.y) + Math.abs(this.z - v.z);

    }

    setFromSpherical(s) {

        return this.setFromSphericalCoords(s.radius, s.phi, s.theta);

    }

    setFromSphericalCoords(radius, phi, theta) {

        const sinPhiRadius = Math.sin(phi) * radius;

        this.x = sinPhiRadius * Math.sin(theta);
        this.y = Math.cos(phi) * radius;
        this.z = sinPhiRadius * Math.cos(theta);

        return this;

    }

    setFromCylindrical(c) {

        return this.setFromCylindricalCoords(c.radius, c.theta, c.y);

    }

    setFromCylindricalCoords(radius, theta, y) {

        this.x = radius * Math.sin(theta);
        this.y = y;
        this.z = radius * Math.cos(theta);

        return this;

    }

    setFromMatrixPosition(m) {

        const e = m.elements;

        this.x = e[12];
        this.y = e[13];
        this.z = e[14];

        return this;

    }

    setFromMatrixScale(m) {

        const sx = this.setFromMatrixColumn(m, 0).length();
        const sy = this.setFromMatrixColumn(m, 1).length();
        const sz = this.setFromMatrixColumn(m, 2).length();

        this.x = sx;
        this.y = sy;
        this.z = sz;

        return this;

    }

    setFromMatrixColumn(m, index) {

        return this.fromArray(m.elements, index * 4);

    }

    setFromMatrix3Column(m, index) {

        return this.fromArray(m.elements, index * 3);

    }

    equals(v) {

        return ((v.x === this.x) && (v.y === this.y) && (v.z === this.z));

    }

    fromArray(array, offset = 0) {

        this.x = array[offset];
        this.y = array[offset + 1];
        this.z = array[offset + 2];

        return this;

    }

    toArray(array = [], offset = 0) {

        array[offset] = this.x;
        array[offset + 1] = this.y;
        array[offset + 2] = this.z;

        return array;

    }

    fromBufferAttribute(attribute, index, offset) {

        if (offset !== undefined) {

            console.warn('THREE.Vector3: offset has been removed from .fromBufferAttribute().');

        }

        this.x = attribute.getX(index);
        this.y = attribute.getY(index);
        this.z = attribute.getZ(index);

        return this;

    }

    random() {

        this.x = Math.random();
        this.y = Math.random();
        this.z = Math.random();

        return this;

    }

}

Vector3.prototype.isVector3 = true;

const _vector$9 = /*@__PURE__*/ new Vector3();
const StaticDrawUsage = 35044;
class BufferAttribute {

    constructor(array, itemSize, normalized) {

        if (Array.isArray(array)) {

            throw new TypeError('THREE.BufferAttribute: array should be a Typed Array.');

        }

        this.name = '';

        this.array = array;
        this.itemSize = itemSize;
        this.count = array !== undefined ? array.length / itemSize : 0;
        this.normalized = normalized === true;

        this.usage = StaticDrawUsage;
        this.updateRange = { offset: 0, count: - 1 };

        this.version = 0;

    }







    applyMatrix4(m) {

        for (let i = 0, l = this.count; i < l; i++) {

            _vector$9.x = this.getX(i);
            _vector$9.y = this.getY(i);
            _vector$9.z = this.getZ(i);

            _vector$9.applyMatrix4(m);

            this.setXYZ(i, _vector$9.x, _vector$9.y, _vector$9.z);

        }

        return this;

    }



    getX(index) {

        return this.array[index * this.itemSize];

    }


    getY(index) {

        return this.array[index * this.itemSize + 1];

    }



    getZ(index) {

        return this.array[index * this.itemSize + 2];

    }


    setXYZ(index, x, y, z) {

        index *= this.itemSize;

        this.array[index + 0] = x;
        this.array[index + 1] = y;
        this.array[index + 2] = z;

        return this;

    }


}


const _vector$c = /*@__PURE__*/ new Vector3();
const _quaternion$4 = /*@__PURE__*/ new Quaternion();


class Matrix4 {

    constructor() {

        this.elements = [

            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1

        ];

        if (arguments.length > 0) {

            console.error('THREE.Matrix4: the constructor no longer reads arguments. use .set() instead.');

        }

    }

    set(n11, n12, n13, n14, n21, n22, n23, n24, n31, n32, n33, n34, n41, n42, n43, n44) {

        const te = this.elements;

        te[0] = n11; te[4] = n12; te[8] = n13; te[12] = n14;
        te[1] = n21; te[5] = n22; te[9] = n23; te[13] = n24;
        te[2] = n31; te[6] = n32; te[10] = n33; te[14] = n34;
        te[3] = n41; te[7] = n42; te[11] = n43; te[15] = n44;

        return this;

    }

    identity() {

        this.set(

            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1

        );

        return this;

    }

    clone() {

        return new Matrix4().fromArray(this.elements);

    }

    copy(m) {

        const te = this.elements;
        const me = m.elements;

        te[0] = me[0]; te[1] = me[1]; te[2] = me[2]; te[3] = me[3];
        te[4] = me[4]; te[5] = me[5]; te[6] = me[6]; te[7] = me[7];
        te[8] = me[8]; te[9] = me[9]; te[10] = me[10]; te[11] = me[11];
        te[12] = me[12]; te[13] = me[13]; te[14] = me[14]; te[15] = me[15];

        return this;

    }

    copyPosition(m) {

        const te = this.elements, me = m.elements;

        te[12] = me[12];
        te[13] = me[13];
        te[14] = me[14];

        return this;

    }

    setFromMatrix3(m) {

        const me = m.elements;

        this.set(

            me[0], me[3], me[6], 0,
            me[1], me[4], me[7], 0,
            me[2], me[5], me[8], 0,
            0, 0, 0, 1

        );

        return this;

    }

    extractBasis(xAxis, yAxis, zAxis) {

        xAxis.setFromMatrixColumn(this, 0);
        yAxis.setFromMatrixColumn(this, 1);
        zAxis.setFromMatrixColumn(this, 2);

        return this;

    }

    makeBasis(xAxis, yAxis, zAxis) {

        this.set(
            xAxis.x, yAxis.x, zAxis.x, 0,
            xAxis.y, yAxis.y, zAxis.y, 0,
            xAxis.z, yAxis.z, zAxis.z, 0,
            0, 0, 0, 1
        );

        return this;

    }

    extractRotation(m) {

        // this method does not support reflection matrices

        const te = this.elements;
        const me = m.elements;

        const scaleX = 1 / _v1$5.setFromMatrixColumn(m, 0).length();
        const scaleY = 1 / _v1$5.setFromMatrixColumn(m, 1).length();
        const scaleZ = 1 / _v1$5.setFromMatrixColumn(m, 2).length();

        te[0] = me[0] * scaleX;
        te[1] = me[1] * scaleX;
        te[2] = me[2] * scaleX;
        te[3] = 0;

        te[4] = me[4] * scaleY;
        te[5] = me[5] * scaleY;
        te[6] = me[6] * scaleY;
        te[7] = 0;

        te[8] = me[8] * scaleZ;
        te[9] = me[9] * scaleZ;
        te[10] = me[10] * scaleZ;
        te[11] = 0;

        te[12] = 0;
        te[13] = 0;
        te[14] = 0;
        te[15] = 1;

        return this;

    }

    makeRotationFromEuler(euler) {

        if (!(euler && euler.isEuler)) {

            console.error('THREE.Matrix4: .makeRotationFromEuler() now expects a Euler rotation rather than a Vector3 and order.');

        }

        const te = this.elements;

        const x = euler.x, y = euler.y, z = euler.z;
        const a = Math.cos(x), b = Math.sin(x);
        const c = Math.cos(y), d = Math.sin(y);
        const e = Math.cos(z), f = Math.sin(z);

        if (euler.order === 'XYZ') {

            const ae = a * e, af = a * f, be = b * e, bf = b * f;

            te[0] = c * e;
            te[4] = - c * f;
            te[8] = d;

            te[1] = af + be * d;
            te[5] = ae - bf * d;
            te[9] = - b * c;

            te[2] = bf - ae * d;
            te[6] = be + af * d;
            te[10] = a * c;

        } else if (euler.order === 'YXZ') {

            const ce = c * e, cf = c * f, de = d * e, df = d * f;

            te[0] = ce + df * b;
            te[4] = de * b - cf;
            te[8] = a * d;

            te[1] = a * f;
            te[5] = a * e;
            te[9] = - b;

            te[2] = cf * b - de;
            te[6] = df + ce * b;
            te[10] = a * c;

        } else if (euler.order === 'ZXY') {

            const ce = c * e, cf = c * f, de = d * e, df = d * f;

            te[0] = ce - df * b;
            te[4] = - a * f;
            te[8] = de + cf * b;

            te[1] = cf + de * b;
            te[5] = a * e;
            te[9] = df - ce * b;

            te[2] = - a * d;
            te[6] = b;
            te[10] = a * c;

        } else if (euler.order === 'ZYX') {

            const ae = a * e, af = a * f, be = b * e, bf = b * f;

            te[0] = c * e;
            te[4] = be * d - af;
            te[8] = ae * d + bf;

            te[1] = c * f;
            te[5] = bf * d + ae;
            te[9] = af * d - be;

            te[2] = - d;
            te[6] = b * c;
            te[10] = a * c;

        } else if (euler.order === 'YZX') {

            const ac = a * c, ad = a * d, bc = b * c, bd = b * d;

            te[0] = c * e;
            te[4] = bd - ac * f;
            te[8] = bc * f + ad;

            te[1] = f;
            te[5] = a * e;
            te[9] = - b * e;

            te[2] = - d * e;
            te[6] = ad * f + bc;
            te[10] = ac - bd * f;

        } else if (euler.order === 'XZY') {

            const ac = a * c, ad = a * d, bc = b * c, bd = b * d;

            te[0] = c * e;
            te[4] = - f;
            te[8] = d * e;

            te[1] = ac * f + bd;
            te[5] = a * e;
            te[9] = ad * f - bc;

            te[2] = bc * f - ad;
            te[6] = b * e;
            te[10] = bd * f + ac;

        }

        // bottom row
        te[3] = 0;
        te[7] = 0;
        te[11] = 0;

        // last column
        te[12] = 0;
        te[13] = 0;
        te[14] = 0;
        te[15] = 1;

        return this;

    }

    makeRotationFromQuaternion(q) {

        return this.compose(_zero, q, _one);

    }

    lookAt(eye, target, up) {

        const te = this.elements;

        _z.subVectors(eye, target);

        if (_z.lengthSq() === 0) {

            // eye and target are in the same position

            _z.z = 1;

        }

        _z.normalize();
        _x.crossVectors(up, _z);

        if (_x.lengthSq() === 0) {

            // up and z are parallel

            if (Math.abs(up.z) === 1) {

                _z.x += 0.0001;

            } else {

                _z.z += 0.0001;

            }

            _z.normalize();
            _x.crossVectors(up, _z);

        }

        _x.normalize();
        _y.crossVectors(_z, _x);

        te[0] = _x.x; te[4] = _y.x; te[8] = _z.x;
        te[1] = _x.y; te[5] = _y.y; te[9] = _z.y;
        te[2] = _x.z; te[6] = _y.z; te[10] = _z.z;

        return this;

    }

    multiply(m, n) {

        if (n !== undefined) {

            console.warn('THREE.Matrix4: .multiply() now only accepts one argument. Use .multiplyMatrices( a, b ) instead.');
            return this.multiplyMatrices(m, n);

        }

        return this.multiplyMatrices(this, m);

    }

    premultiply(m) {

        return this.multiplyMatrices(m, this);

    }

    multiplyMatrices(a, b) {

        const ae = a.elements;
        const be = b.elements;
        const te = this.elements;

        const a11 = ae[0], a12 = ae[4], a13 = ae[8], a14 = ae[12];
        const a21 = ae[1], a22 = ae[5], a23 = ae[9], a24 = ae[13];
        const a31 = ae[2], a32 = ae[6], a33 = ae[10], a34 = ae[14];
        const a41 = ae[3], a42 = ae[7], a43 = ae[11], a44 = ae[15];

        const b11 = be[0], b12 = be[4], b13 = be[8], b14 = be[12];
        const b21 = be[1], b22 = be[5], b23 = be[9], b24 = be[13];
        const b31 = be[2], b32 = be[6], b33 = be[10], b34 = be[14];
        const b41 = be[3], b42 = be[7], b43 = be[11], b44 = be[15];

        te[0] = a11 * b11 + a12 * b21 + a13 * b31 + a14 * b41;
        te[4] = a11 * b12 + a12 * b22 + a13 * b32 + a14 * b42;
        te[8] = a11 * b13 + a12 * b23 + a13 * b33 + a14 * b43;
        te[12] = a11 * b14 + a12 * b24 + a13 * b34 + a14 * b44;

        te[1] = a21 * b11 + a22 * b21 + a23 * b31 + a24 * b41;
        te[5] = a21 * b12 + a22 * b22 + a23 * b32 + a24 * b42;
        te[9] = a21 * b13 + a22 * b23 + a23 * b33 + a24 * b43;
        te[13] = a21 * b14 + a22 * b24 + a23 * b34 + a24 * b44;

        te[2] = a31 * b11 + a32 * b21 + a33 * b31 + a34 * b41;
        te[6] = a31 * b12 + a32 * b22 + a33 * b32 + a34 * b42;
        te[10] = a31 * b13 + a32 * b23 + a33 * b33 + a34 * b43;
        te[14] = a31 * b14 + a32 * b24 + a33 * b34 + a34 * b44;

        te[3] = a41 * b11 + a42 * b21 + a43 * b31 + a44 * b41;
        te[7] = a41 * b12 + a42 * b22 + a43 * b32 + a44 * b42;
        te[11] = a41 * b13 + a42 * b23 + a43 * b33 + a44 * b43;
        te[15] = a41 * b14 + a42 * b24 + a43 * b34 + a44 * b44;

        return this;

    }

    multiplyScalar(s) {

        const te = this.elements;

        te[0] *= s; te[4] *= s; te[8] *= s; te[12] *= s;
        te[1] *= s; te[5] *= s; te[9] *= s; te[13] *= s;
        te[2] *= s; te[6] *= s; te[10] *= s; te[14] *= s;
        te[3] *= s; te[7] *= s; te[11] *= s; te[15] *= s;

        return this;

    }

    determinant() {

        const te = this.elements;

        const n11 = te[0], n12 = te[4], n13 = te[8], n14 = te[12];
        const n21 = te[1], n22 = te[5], n23 = te[9], n24 = te[13];
        const n31 = te[2], n32 = te[6], n33 = te[10], n34 = te[14];
        const n41 = te[3], n42 = te[7], n43 = te[11], n44 = te[15];

        //TODO: make this more efficient
        //( based on http://www.euclideanspace.com/maths/algebra/matrix/functions/inverse/fourD/index.htm )

        return (
            n41 * (
                + n14 * n23 * n32
                - n13 * n24 * n32
                - n14 * n22 * n33
                + n12 * n24 * n33
                + n13 * n22 * n34
                - n12 * n23 * n34
            ) +
            n42 * (
                + n11 * n23 * n34
                - n11 * n24 * n33
                + n14 * n21 * n33
                - n13 * n21 * n34
                + n13 * n24 * n31
                - n14 * n23 * n31
            ) +
            n43 * (
                + n11 * n24 * n32
                - n11 * n22 * n34
                - n14 * n21 * n32
                + n12 * n21 * n34
                + n14 * n22 * n31
                - n12 * n24 * n31
            ) +
            n44 * (
                - n13 * n22 * n31
                - n11 * n23 * n32
                + n11 * n22 * n33
                + n13 * n21 * n32
                - n12 * n21 * n33
                + n12 * n23 * n31
            )

        );

    }

    transpose() {

        const te = this.elements;
        let tmp;

        tmp = te[1]; te[1] = te[4]; te[4] = tmp;
        tmp = te[2]; te[2] = te[8]; te[8] = tmp;
        tmp = te[6]; te[6] = te[9]; te[9] = tmp;

        tmp = te[3]; te[3] = te[12]; te[12] = tmp;
        tmp = te[7]; te[7] = te[13]; te[13] = tmp;
        tmp = te[11]; te[11] = te[14]; te[14] = tmp;

        return this;

    }

    setPosition(x, y, z) {

        const te = this.elements;

        if (x.isVector3) {

            te[12] = x.x;
            te[13] = x.y;
            te[14] = x.z;

        } else {

            te[12] = x;
            te[13] = y;
            te[14] = z;

        }

        return this;

    }

    invert() {

        // based on http://www.euclideanspace.com/maths/algebra/matrix/functions/inverse/fourD/index.htm
        const te = this.elements,

            n11 = te[0], n21 = te[1], n31 = te[2], n41 = te[3],
            n12 = te[4], n22 = te[5], n32 = te[6], n42 = te[7],
            n13 = te[8], n23 = te[9], n33 = te[10], n43 = te[11],
            n14 = te[12], n24 = te[13], n34 = te[14], n44 = te[15],

            t11 = n23 * n34 * n42 - n24 * n33 * n42 + n24 * n32 * n43 - n22 * n34 * n43 - n23 * n32 * n44 + n22 * n33 * n44,
            t12 = n14 * n33 * n42 - n13 * n34 * n42 - n14 * n32 * n43 + n12 * n34 * n43 + n13 * n32 * n44 - n12 * n33 * n44,
            t13 = n13 * n24 * n42 - n14 * n23 * n42 + n14 * n22 * n43 - n12 * n24 * n43 - n13 * n22 * n44 + n12 * n23 * n44,
            t14 = n14 * n23 * n32 - n13 * n24 * n32 - n14 * n22 * n33 + n12 * n24 * n33 + n13 * n22 * n34 - n12 * n23 * n34;

        const det = n11 * t11 + n21 * t12 + n31 * t13 + n41 * t14;

        if (det === 0) return this.set(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

        const detInv = 1 / det;

        te[0] = t11 * detInv;
        te[1] = (n24 * n33 * n41 - n23 * n34 * n41 - n24 * n31 * n43 + n21 * n34 * n43 + n23 * n31 * n44 - n21 * n33 * n44) * detInv;
        te[2] = (n22 * n34 * n41 - n24 * n32 * n41 + n24 * n31 * n42 - n21 * n34 * n42 - n22 * n31 * n44 + n21 * n32 * n44) * detInv;
        te[3] = (n23 * n32 * n41 - n22 * n33 * n41 - n23 * n31 * n42 + n21 * n33 * n42 + n22 * n31 * n43 - n21 * n32 * n43) * detInv;

        te[4] = t12 * detInv;
        te[5] = (n13 * n34 * n41 - n14 * n33 * n41 + n14 * n31 * n43 - n11 * n34 * n43 - n13 * n31 * n44 + n11 * n33 * n44) * detInv;
        te[6] = (n14 * n32 * n41 - n12 * n34 * n41 - n14 * n31 * n42 + n11 * n34 * n42 + n12 * n31 * n44 - n11 * n32 * n44) * detInv;
        te[7] = (n12 * n33 * n41 - n13 * n32 * n41 + n13 * n31 * n42 - n11 * n33 * n42 - n12 * n31 * n43 + n11 * n32 * n43) * detInv;

        te[8] = t13 * detInv;
        te[9] = (n14 * n23 * n41 - n13 * n24 * n41 - n14 * n21 * n43 + n11 * n24 * n43 + n13 * n21 * n44 - n11 * n23 * n44) * detInv;
        te[10] = (n12 * n24 * n41 - n14 * n22 * n41 + n14 * n21 * n42 - n11 * n24 * n42 - n12 * n21 * n44 + n11 * n22 * n44) * detInv;
        te[11] = (n13 * n22 * n41 - n12 * n23 * n41 - n13 * n21 * n42 + n11 * n23 * n42 + n12 * n21 * n43 - n11 * n22 * n43) * detInv;

        te[12] = t14 * detInv;
        te[13] = (n13 * n24 * n31 - n14 * n23 * n31 + n14 * n21 * n33 - n11 * n24 * n33 - n13 * n21 * n34 + n11 * n23 * n34) * detInv;
        te[14] = (n14 * n22 * n31 - n12 * n24 * n31 - n14 * n21 * n32 + n11 * n24 * n32 + n12 * n21 * n34 - n11 * n22 * n34) * detInv;
        te[15] = (n12 * n23 * n31 - n13 * n22 * n31 + n13 * n21 * n32 - n11 * n23 * n32 - n12 * n21 * n33 + n11 * n22 * n33) * detInv;

        return this;

    }

    scale(v) {

        const te = this.elements;
        const x = v.x, y = v.y, z = v.z;

        te[0] *= x; te[4] *= y; te[8] *= z;
        te[1] *= x; te[5] *= y; te[9] *= z;
        te[2] *= x; te[6] *= y; te[10] *= z;
        te[3] *= x; te[7] *= y; te[11] *= z;

        return this;

    }

    getMaxScaleOnAxis() {

        const te = this.elements;

        const scaleXSq = te[0] * te[0] + te[1] * te[1] + te[2] * te[2];
        const scaleYSq = te[4] * te[4] + te[5] * te[5] + te[6] * te[6];
        const scaleZSq = te[8] * te[8] + te[9] * te[9] + te[10] * te[10];

        return Math.sqrt(Math.max(scaleXSq, scaleYSq, scaleZSq));

    }

    makeTranslation(x, y, z) {

        this.set(

            1, 0, 0, x,
            0, 1, 0, y,
            0, 0, 1, z,
            0, 0, 0, 1

        );

        return this;

    }

    makeRotationX(theta) {

        const c = Math.cos(theta), s = Math.sin(theta);

        this.set(

            1, 0, 0, 0,
            0, c, - s, 0,
            0, s, c, 0,
            0, 0, 0, 1

        );

        return this;

    }

    makeRotationY(theta) {

        const c = Math.cos(theta), s = Math.sin(theta);

        this.set(

            c, 0, s, 0,
            0, 1, 0, 0,
            - s, 0, c, 0,
            0, 0, 0, 1

        );

        return this;

    }

    makeRotationZ(theta) {

        const c = Math.cos(theta), s = Math.sin(theta);

        this.set(

            c, - s, 0, 0,
            s, c, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1

        );

        return this;

    }

    makeRotationAxis(axis, angle) {

        // Based on http://www.gamedev.net/reference/articles/article1199.asp

        const c = Math.cos(angle);
        const s = Math.sin(angle);
        const t = 1 - c;
        const x = axis.x, y = axis.y, z = axis.z;
        const tx = t * x, ty = t * y;

        this.set(

            tx * x + c, tx * y - s * z, tx * z + s * y, 0,
            tx * y + s * z, ty * y + c, ty * z - s * x, 0,
            tx * z - s * y, ty * z + s * x, t * z * z + c, 0,
            0, 0, 0, 1

        );

        return this;

    }

    makeScale(x, y, z) {

        this.set(

            x, 0, 0, 0,
            0, y, 0, 0,
            0, 0, z, 0,
            0, 0, 0, 1

        );

        return this;

    }

    makeShear(xy, xz, yx, yz, zx, zy) {

        this.set(

            1, yx, zx, 0,
            xy, 1, zy, 0,
            xz, yz, 1, 0,
            0, 0, 0, 1

        );

        return this;

    }

    compose(position, quaternion, scale) {

        const te = this.elements;

        const x = quaternion._x, y = quaternion._y, z = quaternion._z, w = quaternion._w;
        const x2 = x + x, y2 = y + y, z2 = z + z;
        const xx = x * x2, xy = x * y2, xz = x * z2;
        const yy = y * y2, yz = y * z2, zz = z * z2;
        const wx = w * x2, wy = w * y2, wz = w * z2;

        const sx = scale.x, sy = scale.y, sz = scale.z;

        te[0] = (1 - (yy + zz)) * sx;
        te[1] = (xy + wz) * sx;
        te[2] = (xz - wy) * sx;
        te[3] = 0;

        te[4] = (xy - wz) * sy;
        te[5] = (1 - (xx + zz)) * sy;
        te[6] = (yz + wx) * sy;
        te[7] = 0;

        te[8] = (xz + wy) * sz;
        te[9] = (yz - wx) * sz;
        te[10] = (1 - (xx + yy)) * sz;
        te[11] = 0;

        te[12] = position.x;
        te[13] = position.y;
        te[14] = position.z;
        te[15] = 1;

        return this;

    }

    decompose(position, quaternion, scale) {

        const te = this.elements;

        let sx = _v1$5.set(te[0], te[1], te[2]).length();
        const sy = _v1$5.set(te[4], te[5], te[6]).length();
        const sz = _v1$5.set(te[8], te[9], te[10]).length();

        // if determine is negative, we need to invert one scale
        const det = this.determinant();
        if (det < 0) sx = - sx;

        position.x = te[12];
        position.y = te[13];
        position.z = te[14];

        // scale the rotation part
        _m1$2.copy(this);

        const invSX = 1 / sx;
        const invSY = 1 / sy;
        const invSZ = 1 / sz;

        _m1$2.elements[0] *= invSX;
        _m1$2.elements[1] *= invSX;
        _m1$2.elements[2] *= invSX;

        _m1$2.elements[4] *= invSY;
        _m1$2.elements[5] *= invSY;
        _m1$2.elements[6] *= invSY;

        _m1$2.elements[8] *= invSZ;
        _m1$2.elements[9] *= invSZ;
        _m1$2.elements[10] *= invSZ;

        quaternion.setFromRotationMatrix(_m1$2);

        scale.x = sx;
        scale.y = sy;
        scale.z = sz;

        return this;

    }

    makePerspective(left, right, top, bottom, near, far) {

        if (far === undefined) {

            console.warn('THREE.Matrix4: .makePerspective() has been redefined and has a new signature. Please check the docs.');

        }

        const te = this.elements;
        const x = 2 * near / (right - left);
        const y = 2 * near / (top - bottom);

        const a = (right + left) / (right - left);
        const b = (top + bottom) / (top - bottom);
        const c = - (far + near) / (far - near);
        const d = - 2 * far * near / (far - near);

        te[0] = x; te[4] = 0; te[8] = a; te[12] = 0;
        te[1] = 0; te[5] = y; te[9] = b; te[13] = 0;
        te[2] = 0; te[6] = 0; te[10] = c; te[14] = d;
        te[3] = 0; te[7] = 0; te[11] = - 1; te[15] = 0;

        return this;

    }

    makeOrthographic(left, right, top, bottom, near, far) {

        const te = this.elements;
        const w = 1.0 / (right - left);
        const h = 1.0 / (top - bottom);
        const p = 1.0 / (far - near);

        const x = (right + left) * w;
        const y = (top + bottom) * h;
        const z = (far + near) * p;

        te[0] = 2 * w; te[4] = 0; te[8] = 0; te[12] = - x;
        te[1] = 0; te[5] = 2 * h; te[9] = 0; te[13] = - y;
        te[2] = 0; te[6] = 0; te[10] = - 2 * p; te[14] = - z;
        te[3] = 0; te[7] = 0; te[11] = 0; te[15] = 1;

        return this;

    }

    equals(matrix) {

        const te = this.elements;
        const me = matrix.elements;

        for (let i = 0; i < 16; i++) {

            if (te[i] !== me[i]) return false;

        }

        return true;

    }

    fromArray(array, offset = 0) {

        for (let i = 0; i < 16; i++) {

            this.elements[i] = array[i + offset];

        }

        return this;

    }

    toArray(array = [], offset = 0) {

        const te = this.elements;

        array[offset] = te[0];
        array[offset + 1] = te[1];
        array[offset + 2] = te[2];
        array[offset + 3] = te[3];

        array[offset + 4] = te[4];
        array[offset + 5] = te[5];
        array[offset + 6] = te[6];
        array[offset + 7] = te[7];

        array[offset + 8] = te[8];
        array[offset + 9] = te[9];
        array[offset + 10] = te[10];
        array[offset + 11] = te[11];

        array[offset + 12] = te[12];
        array[offset + 13] = te[13];
        array[offset + 14] = te[14];
        array[offset + 15] = te[15];

        return array;

    }

}

Matrix4.prototype.isMatrix4 = true;

const _v1$5 = /*@__PURE__*/ new Vector3();
const _m1$2 = /*@__PURE__*/ new Matrix4();
const _zero = /*@__PURE__*/ new Vector3(0, 0, 0);
const _one = /*@__PURE__*/ new Vector3(1, 1, 1);
const _x = /*@__PURE__*/ new Vector3();
const _y = /*@__PURE__*/ new Vector3();
const _z = /*@__PURE__*/ new Vector3();

const _matrix$1 = /*@__PURE__*/ new Matrix4();
const _quaternion$3 = /*@__PURE__*/ new Quaternion();

class Box3 {

    constructor(min = new Vector3(+ Infinity, + Infinity, + Infinity), max = new Vector3(- Infinity, - Infinity, - Infinity)) {

        this.min = min;
        this.max = max;

    }

    set(min, max) {

        this.min.copy(min);
        this.max.copy(max);

        return this;

    }

    setFromArray(array) {

        let minX = + Infinity;
        let minY = + Infinity;
        let minZ = + Infinity;

        let maxX = - Infinity;
        let maxY = - Infinity;
        let maxZ = - Infinity;

        for (let i = 0, l = array.length; i < l; i += 3) {

            const x = array[i];
            const y = array[i + 1];
            const z = array[i + 2];

            if (x < minX) minX = x;
            if (y < minY) minY = y;
            if (z < minZ) minZ = z;

            if (x > maxX) maxX = x;
            if (y > maxY) maxY = y;
            if (z > maxZ) maxZ = z;

        }

        this.min.set(minX, minY, minZ);
        this.max.set(maxX, maxY, maxZ);

        return this;

    }

    setFromBufferAttribute(attribute) {

        let minX = + Infinity;
        let minY = + Infinity;
        let minZ = + Infinity;

        let maxX = - Infinity;
        let maxY = - Infinity;
        let maxZ = - Infinity;

        for (let i = 0, l = attribute.count; i < l; i++) {

            const x = attribute.getX(i);
            const y = attribute.getY(i);
            const z = attribute.getZ(i);

            if (x < minX) minX = x;
            if (y < minY) minY = y;
            if (z < minZ) minZ = z;

            if (x > maxX) maxX = x;
            if (y > maxY) maxY = y;
            if (z > maxZ) maxZ = z;

        }

        this.min.set(minX, minY, minZ);
        this.max.set(maxX, maxY, maxZ);

        return this;

    }

    setFromPoints(points) {

        this.makeEmpty();

        for (let i = 0, il = points.length; i < il; i++) {

            this.expandByPoint(points[i]);

        }

        return this;

    }

    setFromCenterAndSize(center, size) {

        const halfSize = _vector$c.copy(size).multiplyScalar(0.5);

        this.min.copy(center).sub(halfSize);
        this.max.copy(center).add(halfSize);

        return this;

    }

    setFromObject(object) {

        this.makeEmpty();

        return this.expandByObject(object);

    }

    clone() {

        return new this.constructor().copy(this);

    }

    copy(box) {

        this.min.copy(box.min);
        this.max.copy(box.max);

        return this;

    }

    makeEmpty() {

        this.min.x = this.min.y = this.min.z = + Infinity;
        this.max.x = this.max.y = this.max.z = - Infinity;

        return this;

    }

    isEmpty() {

        // this is a more robust check for empty than ( volume <= 0 ) because volume can get positive with two negative axes

        return (this.max.x < this.min.x) || (this.max.y < this.min.y) || (this.max.z < this.min.z);

    }

    getCenter(target) {

        return this.isEmpty() ? target.set(0, 0, 0) : target.addVectors(this.min, this.max).multiplyScalar(0.5);

    }

    getSize(target) {

        return this.isEmpty() ? target.set(0, 0, 0) : target.subVectors(this.max, this.min);

    }

    expandByPoint(point) {

        this.min.min(point);
        this.max.max(point);

        return this;

    }

    expandByVector(vector) {

        this.min.sub(vector);
        this.max.add(vector);

        return this;

    }

    expandByScalar(scalar) {

        this.min.addScalar(- scalar);
        this.max.addScalar(scalar);

        return this;

    }

    expandByObject(object) {

        // Computes the world-axis-aligned bounding box of an object (including its children),
        // accounting for both the object's, and children's, world transforms

        object.updateWorldMatrix(false, false);

        const geometry = object.geometry;

        if (geometry !== undefined) {

            if (geometry.boundingBox === null) {

                geometry.computeBoundingBox();

            }

            _box$3.copy(geometry.boundingBox);
            _box$3.applyMatrix4(object.matrixWorld);

            this.union(_box$3);

        }

        const children = object.children;

        for (let i = 0, l = children.length; i < l; i++) {

            this.expandByObject(children[i]);

        }

        return this;

    }

    containsPoint(point) {

        return point.x < this.min.x || point.x > this.max.x ||
            point.y < this.min.y || point.y > this.max.y ||
            point.z < this.min.z || point.z > this.max.z ? false : true;

    }

    containsBox(box) {

        return this.min.x <= box.min.x && box.max.x <= this.max.x &&
            this.min.y <= box.min.y && box.max.y <= this.max.y &&
            this.min.z <= box.min.z && box.max.z <= this.max.z;

    }

    getParameter(point, target) {

        // This can potentially have a divide by zero if the box
        // has a size dimension of 0.

        return target.set(
            (point.x - this.min.x) / (this.max.x - this.min.x),
            (point.y - this.min.y) / (this.max.y - this.min.y),
            (point.z - this.min.z) / (this.max.z - this.min.z)
        );

    }

    intersectsBox(box) {

        // using 6 splitting planes to rule out intersections.
        return box.max.x < this.min.x || box.min.x > this.max.x ||
            box.max.y < this.min.y || box.min.y > this.max.y ||
            box.max.z < this.min.z || box.min.z > this.max.z ? false : true;

    }

    intersectsSphere(sphere) {

        // Find the point on the AABB closest to the sphere center.
        this.clampPoint(sphere.center, _vector$c);

        // If that point is inside the sphere, the AABB and sphere intersect.
        return _vector$c.distanceToSquared(sphere.center) <= (sphere.radius * sphere.radius);

    }

    intersectsPlane(plane) {

        // We compute the minimum and maximum dot product values. If those values
        // are on the same side (back or front) of the plane, then there is no intersection.

        let min, max;

        if (plane.normal.x > 0) {

            min = plane.normal.x * this.min.x;
            max = plane.normal.x * this.max.x;

        } else {

            min = plane.normal.x * this.max.x;
            max = plane.normal.x * this.min.x;

        }

        if (plane.normal.y > 0) {

            min += plane.normal.y * this.min.y;
            max += plane.normal.y * this.max.y;

        } else {

            min += plane.normal.y * this.max.y;
            max += plane.normal.y * this.min.y;

        }

        if (plane.normal.z > 0) {

            min += plane.normal.z * this.min.z;
            max += plane.normal.z * this.max.z;

        } else {

            min += plane.normal.z * this.max.z;
            max += plane.normal.z * this.min.z;

        }

        return (min <= - plane.constant && max >= - plane.constant);

    }

    intersectsTriangle(triangle) {

        if (this.isEmpty()) {

            return false;

        }

        // compute box center and extents
        this.getCenter(_center);
        _extents.subVectors(this.max, _center);

        // translate triangle to aabb origin
        _v0$2.subVectors(triangle.a, _center);
        _v1$7.subVectors(triangle.b, _center);
        _v2$3.subVectors(triangle.c, _center);

        // compute edge vectors for triangle
        _f0.subVectors(_v1$7, _v0$2);
        _f1.subVectors(_v2$3, _v1$7);
        _f2.subVectors(_v0$2, _v2$3);

        // test against axes that are given by cross product combinations of the edges of the triangle and the edges of the aabb
        // make an axis testing of each of the 3 sides of the aabb against each of the 3 sides of the triangle = 9 axis of separation
        // axis_ij = u_i x f_j (u0, u1, u2 = face normals of aabb = x,y,z axes vectors since aabb is axis aligned)
        let axes = [
            0, - _f0.z, _f0.y, 0, - _f1.z, _f1.y, 0, - _f2.z, _f2.y,
            _f0.z, 0, - _f0.x, _f1.z, 0, - _f1.x, _f2.z, 0, - _f2.x,
            - _f0.y, _f0.x, 0, - _f1.y, _f1.x, 0, - _f2.y, _f2.x, 0
        ];
        if (!satForAxes(axes, _v0$2, _v1$7, _v2$3, _extents)) {

            return false;

        }

        // test 3 face normals from the aabb
        axes = [1, 0, 0, 0, 1, 0, 0, 0, 1];
        if (!satForAxes(axes, _v0$2, _v1$7, _v2$3, _extents)) {

            return false;

        }

        // finally testing the face normal of the triangle
        // use already existing triangle edge vectors here
        _triangleNormal.crossVectors(_f0, _f1);
        axes = [_triangleNormal.x, _triangleNormal.y, _triangleNormal.z];

        return satForAxes(axes, _v0$2, _v1$7, _v2$3, _extents);

    }

    clampPoint(point, target) {

        return target.copy(point).clamp(this.min, this.max);

    }

    distanceToPoint(point) {

        const clampedPoint = _vector$c.copy(point).clamp(this.min, this.max);

        return clampedPoint.sub(point).length();

    }

    getBoundingSphere(target) {

        this.getCenter(target.center);

        target.radius = this.getSize(_vector$c).length() * 0.5;

        return target;

    }

    intersect(box) {

        this.min.max(box.min);
        this.max.min(box.max);

        // ensure that if there is no overlap, the result is fully empty, not slightly empty with non-inf/+inf values that will cause subsequence intersects to erroneously return valid values.
        if (this.isEmpty()) this.makeEmpty();

        return this;

    }

    union(box) {

        this.min.min(box.min);
        this.max.max(box.max);

        return this;

    }

    applyMatrix4(matrix) {

        // transform of empty box is an empty box.
        if (this.isEmpty()) return this;

        // NOTE: I am using a binary pattern to specify all 2^3 combinations below
        _points[0].set(this.min.x, this.min.y, this.min.z).applyMatrix4(matrix); // 000
        _points[1].set(this.min.x, this.min.y, this.max.z).applyMatrix4(matrix); // 001
        _points[2].set(this.min.x, this.max.y, this.min.z).applyMatrix4(matrix); // 010
        _points[3].set(this.min.x, this.max.y, this.max.z).applyMatrix4(matrix); // 011
        _points[4].set(this.max.x, this.min.y, this.min.z).applyMatrix4(matrix); // 100
        _points[5].set(this.max.x, this.min.y, this.max.z).applyMatrix4(matrix); // 101
        _points[6].set(this.max.x, this.max.y, this.min.z).applyMatrix4(matrix); // 110
        _points[7].set(this.max.x, this.max.y, this.max.z).applyMatrix4(matrix); // 111

        this.setFromPoints(_points);

        return this;

    }

    translate(offset) {

        this.min.add(offset);
        this.max.add(offset);

        return this;

    }

    equals(box) {

        return box.min.equals(this.min) && box.max.equals(this.max);

    }

}

Box3.prototype.isBox3 = true;
const _box$2 = /*@__PURE__*/ new Box3();
const _v1$6 = /*@__PURE__*/ new Vector3();
const _toFarthestPoint = /*@__PURE__*/ new Vector3();
const _toPoint = /*@__PURE__*/ new Vector3();

class Sphere {

    constructor(center = new Vector3(), radius = - 1) {

        this.center = center;
        this.radius = radius;

    }

    set(center, radius) {

        this.center.copy(center);
        this.radius = radius;

        return this;

    }

    setFromPoints(points, optionalCenter) {

        const center = this.center;

        if (optionalCenter !== undefined) {

            center.copy(optionalCenter);

        } else {

            _box$2.setFromPoints(points).getCenter(center);

        }

        let maxRadiusSq = 0;

        for (let i = 0, il = points.length; i < il; i++) {

            maxRadiusSq = Math.max(maxRadiusSq, center.distanceToSquared(points[i]));

        }

        this.radius = Math.sqrt(maxRadiusSq);

        return this;

    }

    copy(sphere) {

        this.center.copy(sphere.center);
        this.radius = sphere.radius;

        return this;

    }

    isEmpty() {

        return (this.radius < 0);

    }

    makeEmpty() {

        this.center.set(0, 0, 0);
        this.radius = - 1;

        return this;

    }

    containsPoint(point) {

        return (point.distanceToSquared(this.center) <= (this.radius * this.radius));

    }

    distanceToPoint(point) {

        return (point.distanceTo(this.center) - this.radius);

    }

    intersectsSphere(sphere) {

        const radiusSum = this.radius + sphere.radius;

        return sphere.center.distanceToSquared(this.center) <= (radiusSum * radiusSum);

    }

    intersectsBox(box) {

        return box.intersectsSphere(this);

    }

    intersectsPlane(plane) {

        return Math.abs(plane.distanceToPoint(this.center)) <= this.radius;

    }

    clampPoint(point, target) {

        const deltaLengthSq = this.center.distanceToSquared(point);

        target.copy(point);

        if (deltaLengthSq > (this.radius * this.radius)) {

            target.sub(this.center).normalize();
            target.multiplyScalar(this.radius).add(this.center);

        }

        return target;

    }

    getBoundingBox(target) {

        if (this.isEmpty()) {

            // Empty sphere produces empty bounding box
            target.makeEmpty();
            return target;

        }

        target.set(this.center, this.center);
        target.expandByScalar(this.radius);

        return target;

    }

    applyMatrix4(matrix) {

        this.center.applyMatrix4(matrix);
        this.radius = this.radius * matrix.getMaxScaleOnAxis();

        return this;

    }

    translate(offset) {

        this.center.add(offset);

        return this;

    }

    expandByPoint(point) {

        // from https://github.com/juj/MathGeoLib/blob/2940b99b99cfe575dd45103ef20f4019dee15b54/src/Geometry/Sphere.cpp#L649-L671

        _toPoint.subVectors(point, this.center);

        const lengthSq = _toPoint.lengthSq();

        if (lengthSq > (this.radius * this.radius)) {

            const length = Math.sqrt(lengthSq);
            const missingRadiusHalf = (length - this.radius) * 0.5;

            // Nudge this sphere towards the target point. Add half the missing distance to radius,
            // and the other half to position. This gives a tighter enclosure, instead of if
            // the whole missing distance were just added to radius.

            this.center.add(_toPoint.multiplyScalar(missingRadiusHalf / length));
            this.radius += missingRadiusHalf;

        }

        return this;

    }

    union(sphere) {

        // from https://github.com/juj/MathGeoLib/blob/2940b99b99cfe575dd45103ef20f4019dee15b54/src/Geometry/Sphere.cpp#L759-L769

        // To enclose another sphere into this sphere, we only need to enclose two points:
        // 1) Enclose the farthest point on the other sphere into this sphere.
        // 2) Enclose the opposite point of the farthest point into this sphere.

        _toFarthestPoint.subVectors(sphere.center, this.center).normalize().multiplyScalar(sphere.radius);

        this.expandByPoint(_v1$6.copy(sphere.center).add(_toFarthestPoint));
        this.expandByPoint(_v1$6.copy(sphere.center).sub(_toFarthestPoint));

        return this;

    }

    equals(sphere) {

        return sphere.center.equals(this.center) && (sphere.radius === this.radius);

    }

    clone() {

        return new this.constructor().copy(this);

    }

}

const _vector$a = /*@__PURE__*/ new Vector3();
const _segCenter = /*@__PURE__*/ new Vector3();
const _segDir = /*@__PURE__*/ new Vector3();
const _diff = /*@__PURE__*/ new Vector3();

const _edge1 = /*@__PURE__*/ new Vector3();
const _edge2 = /*@__PURE__*/ new Vector3();
const _normal$1 = /*@__PURE__*/ new Vector3();

class Ray {

    constructor(origin = new Vector3(), direction = new Vector3(0, 0, - 1)) {

        this.origin = origin;
        this.direction = direction;

    }

    set(origin, direction) {

        this.origin.copy(origin);
        this.direction.copy(direction);

        return this;

    }

    copy(ray) {

        this.origin.copy(ray.origin);
        this.direction.copy(ray.direction);

        return this;

    }

    at(t, target) {

        return target.copy(this.direction).multiplyScalar(t).add(this.origin);

    }

    lookAt(v) {

        this.direction.copy(v).sub(this.origin).normalize();

        return this;

    }

    recast(t) {

        this.origin.copy(this.at(t, _vector$a));

        return this;

    }

    closestPointToPoint(point, target) {

        target.subVectors(point, this.origin);

        const directionDistance = target.dot(this.direction);

        if (directionDistance < 0) {

            return target.copy(this.origin);

        }

        return target.copy(this.direction).multiplyScalar(directionDistance).add(this.origin);

    }

    distanceToPoint(point) {

        return Math.sqrt(this.distanceSqToPoint(point));

    }

    distanceSqToPoint(point) {

        const directionDistance = _vector$a.subVectors(point, this.origin).dot(this.direction);

        // point behind the ray

        if (directionDistance < 0) {

            return this.origin.distanceToSquared(point);

        }

        _vector$a.copy(this.direction).multiplyScalar(directionDistance).add(this.origin);

        return _vector$a.distanceToSquared(point);

    }

    distanceSqToSegment(v0, v1, optionalPointOnRay, optionalPointOnSegment) {

        // from http://www.geometrictools.com/GTEngine/Include/Mathematics/GteDistRaySegment.h
        // It returns the min distance between the ray and the segment
        // defined by v0 and v1
        // It can also set two optional targets :
        // - The closest point on the ray
        // - The closest point on the segment

        _segCenter.copy(v0).add(v1).multiplyScalar(0.5);
        _segDir.copy(v1).sub(v0).normalize();
        _diff.copy(this.origin).sub(_segCenter);

        const segExtent = v0.distanceTo(v1) * 0.5;
        const a01 = - this.direction.dot(_segDir);
        const b0 = _diff.dot(this.direction);
        const b1 = - _diff.dot(_segDir);
        const c = _diff.lengthSq();
        const det = Math.abs(1 - a01 * a01);
        let s0, s1, sqrDist, extDet;

        if (det > 0) {

            // The ray and segment are not parallel.

            s0 = a01 * b1 - b0;
            s1 = a01 * b0 - b1;
            extDet = segExtent * det;

            if (s0 >= 0) {

                if (s1 >= - extDet) {

                    if (s1 <= extDet) {

                        // region 0
                        // Minimum at interior points of ray and segment.

                        const invDet = 1 / det;
                        s0 *= invDet;
                        s1 *= invDet;
                        sqrDist = s0 * (s0 + a01 * s1 + 2 * b0) + s1 * (a01 * s0 + s1 + 2 * b1) + c;

                    } else {

                        // region 1

                        s1 = segExtent;
                        s0 = Math.max(0, - (a01 * s1 + b0));
                        sqrDist = - s0 * s0 + s1 * (s1 + 2 * b1) + c;

                    }

                } else {

                    // region 5

                    s1 = - segExtent;
                    s0 = Math.max(0, - (a01 * s1 + b0));
                    sqrDist = - s0 * s0 + s1 * (s1 + 2 * b1) + c;

                }

            } else {

                if (s1 <= - extDet) {

                    // region 4

                    s0 = Math.max(0, - (- a01 * segExtent + b0));
                    s1 = (s0 > 0) ? - segExtent : Math.min(Math.max(- segExtent, - b1), segExtent);
                    sqrDist = - s0 * s0 + s1 * (s1 + 2 * b1) + c;

                } else if (s1 <= extDet) {

                    // region 3

                    s0 = 0;
                    s1 = Math.min(Math.max(- segExtent, - b1), segExtent);
                    sqrDist = s1 * (s1 + 2 * b1) + c;

                } else {

                    // region 2

                    s0 = Math.max(0, - (a01 * segExtent + b0));
                    s1 = (s0 > 0) ? segExtent : Math.min(Math.max(- segExtent, - b1), segExtent);
                    sqrDist = - s0 * s0 + s1 * (s1 + 2 * b1) + c;

                }

            }

        } else {

            // Ray and segment are parallel.

            s1 = (a01 > 0) ? - segExtent : segExtent;
            s0 = Math.max(0, - (a01 * s1 + b0));
            sqrDist = - s0 * s0 + s1 * (s1 + 2 * b1) + c;

        }

        if (optionalPointOnRay) {

            optionalPointOnRay.copy(this.direction).multiplyScalar(s0).add(this.origin);

        }

        if (optionalPointOnSegment) {

            optionalPointOnSegment.copy(_segDir).multiplyScalar(s1).add(_segCenter);

        }

        return sqrDist;

    }

    intersectSphere(sphere, target) {

        _vector$a.subVectors(sphere.center, this.origin);
        const tca = _vector$a.dot(this.direction);
        const d2 = _vector$a.dot(_vector$a) - tca * tca;
        const radius2 = sphere.radius * sphere.radius;

        if (d2 > radius2) return null;

        const thc = Math.sqrt(radius2 - d2);

        // t0 = first intersect point - entrance on front of sphere
        const t0 = tca - thc;

        // t1 = second intersect point - exit point on back of sphere
        const t1 = tca + thc;

        // test to see if both t0 and t1 are behind the ray - if so, return null
        if (t0 < 0 && t1 < 0) return null;

        // test to see if t0 is behind the ray:
        // if it is, the ray is inside the sphere, so return the second exit point scaled by t1,
        // in order to always return an intersect point that is in front of the ray.
        if (t0 < 0) return this.at(t1, target);

        // else t0 is in front of the ray, so return the first collision point scaled by t0
        return this.at(t0, target);

    }

    intersectsSphere(sphere) {

        return this.distanceSqToPoint(sphere.center) <= (sphere.radius * sphere.radius);

    }

    distanceToPlane(plane) {

        const denominator = plane.normal.dot(this.direction);

        if (denominator === 0) {

            // line is coplanar, return origin
            if (plane.distanceToPoint(this.origin) === 0) {

                return 0;

            }

            // Null is preferable to undefined since undefined means.... it is undefined

            return null;

        }

        const t = - (this.origin.dot(plane.normal) + plane.constant) / denominator;

        // Return if the ray never intersects the plane

        return t >= 0 ? t : null;

    }

    intersectPlane(plane, target) {

        const t = this.distanceToPlane(plane);

        if (t === null) {

            return null;

        }

        return this.at(t, target);

    }

    intersectsPlane(plane) {

        // check if the ray lies on the plane first

        const distToPoint = plane.distanceToPoint(this.origin);

        if (distToPoint === 0) {

            return true;

        }

        const denominator = plane.normal.dot(this.direction);

        if (denominator * distToPoint < 0) {

            return true;

        }

        // ray origin is behind the plane (and is pointing behind it)

        return false;

    }

    intersectBox(box, target) {

        let tmin, tmax, tymin, tymax, tzmin, tzmax;

        const invdirx = 1 / this.direction.x,
            invdiry = 1 / this.direction.y,
            invdirz = 1 / this.direction.z;

        const origin = this.origin;

        if (invdirx >= 0) {

            tmin = (box.min.x - origin.x) * invdirx;
            tmax = (box.max.x - origin.x) * invdirx;

        } else {

            tmin = (box.max.x - origin.x) * invdirx;
            tmax = (box.min.x - origin.x) * invdirx;

        }

        if (invdiry >= 0) {

            tymin = (box.min.y - origin.y) * invdiry;
            tymax = (box.max.y - origin.y) * invdiry;

        } else {

            tymin = (box.max.y - origin.y) * invdiry;
            tymax = (box.min.y - origin.y) * invdiry;

        }

        if ((tmin > tymax) || (tymin > tmax)) return null;

        // These lines also handle the case where tmin or tmax is NaN
        // (result of 0 * Infinity). x !== x returns true if x is NaN

        if (tymin > tmin || tmin !== tmin) tmin = tymin;

        if (tymax < tmax || tmax !== tmax) tmax = tymax;

        if (invdirz >= 0) {

            tzmin = (box.min.z - origin.z) * invdirz;
            tzmax = (box.max.z - origin.z) * invdirz;

        } else {

            tzmin = (box.max.z - origin.z) * invdirz;
            tzmax = (box.min.z - origin.z) * invdirz;

        }

        if ((tmin > tzmax) || (tzmin > tmax)) return null;

        if (tzmin > tmin || tmin !== tmin) tmin = tzmin;

        if (tzmax < tmax || tmax !== tmax) tmax = tzmax;

        //return point closest to the ray (positive side)

        if (tmax < 0) return null;

        return this.at(tmin >= 0 ? tmin : tmax, target);

    }

    intersectsBox(box) {

        return this.intersectBox(box, _vector$a) !== null;

    }

    intersectTriangle(a, b, c, backfaceCulling, target) {

        // Compute the offset origin, edges, and normal.

        // from http://www.geometrictools.com/GTEngine/Include/Mathematics/GteIntrRay3Triangle3.h

        _edge1.subVectors(b, a);
        _edge2.subVectors(c, a);
        _normal$1.crossVectors(_edge1, _edge2);

        // Solve Q + t*D = b1*E1 + b2*E2 (Q = kDiff, D = ray direction,
        // E1 = kEdge1, E2 = kEdge2, N = Cross(E1,E2)) by
        //   |Dot(D,N)|*b1 = sign(Dot(D,N))*Dot(D,Cross(Q,E2))
        //   |Dot(D,N)|*b2 = sign(Dot(D,N))*Dot(D,Cross(E1,Q))
        //   |Dot(D,N)|*t = -sign(Dot(D,N))*Dot(Q,N)
        let DdN = this.direction.dot(_normal$1);
        let sign;

        if (DdN > 0) {

            if (backfaceCulling) return null;
            sign = 1;

        } else if (DdN < 0) {

            sign = - 1;
            DdN = - DdN;

        } else {

            return null;

        }

        _diff.subVectors(this.origin, a);
        const DdQxE2 = sign * this.direction.dot(_edge2.crossVectors(_diff, _edge2));

        // b1 < 0, no intersection
        if (DdQxE2 < 0) {

            return null;

        }

        const DdE1xQ = sign * this.direction.dot(_edge1.cross(_diff));

        // b2 < 0, no intersection
        if (DdE1xQ < 0) {

            return null;

        }

        // b1+b2 > 1, no intersection
        if (DdQxE2 + DdE1xQ > DdN) {

            return null;

        }

        // Line intersects triangle, check if ray does.
        const QdN = - sign * _diff.dot(_normal$1);

        // t < 0, no intersection
        if (QdN < 0) {

            return null;

        }

        // Ray intersects triangle.
        return this.at(QdN / DdN, target);

    }

    applyMatrix4(matrix4) {

        this.origin.applyMatrix4(matrix4);
        this.direction.transformDirection(matrix4);

        return this;

    }

    equals(ray) {

        return ray.origin.equals(this.origin) && ray.direction.equals(this.direction);

    }

    clone() {

        return new this.constructor().copy(this);

    }

}
class Layers {

    constructor() {

        this.mask = 1 | 0;

    }

    set(channel) {

        this.mask = 1 << channel | 0;

    }

    enable(channel) {

        this.mask |= 1 << channel | 0;

    }

    enableAll() {

        this.mask = 0xffffffff | 0;

    }

    toggle(channel) {

        this.mask ^= 1 << channel | 0;

    }

    disable(channel) {

        this.mask &= ~(1 << channel | 0);

    }

    disableAll() {

        this.mask = 0;

    }

    test(layers) {

        return (this.mask & layers.mask) !== 0;

    }

}


const _intersectionPoint = new Vector3();
const _intersectionPointWorld = new Vector3();

class Raycaster {

    constructor(origin, direction, near = 0, far = Infinity) {

        this.ray = new Ray(origin, direction);
        // direction is assumed to be normalized (for accurate distance calculations)

        this.near = near;
        this.far = far;
        this.camera = null;
        this.layers = new Layers();

        this.params = {
            Mesh: {},
            Line: { threshold: 1 },
            LOD: {},
            Points: { threshold: 1 },
            Sprite: {}
        };

    }

    set(origin, direction) {
        this.ray.set(origin, direction);
    }

    setFromCamera(coords, camera) {

        if (camera && camera.isPerspectiveCamera) {

            this.ray.origin.setFromMatrixPosition(camera.matrixWorld);
            this.ray.direction.set(coords.x, coords.y, 0.5).unproject(camera).sub(this.ray.origin).normalize();
            this.camera = camera;

        } else if (camera && camera.isOrthographicCamera) {

            this.ray.origin.set(coords.x, coords.y, (camera.near + camera.far) / (camera.near - camera.far)).unproject(camera); // set origin in plane of camera
            this.ray.direction.set(0, 0, - 1).transformDirection(camera.matrixWorld);
            this.camera = camera;

        } else {

            console.error('THREE.Raycaster: Unsupported camera type: ' + camera.type);

        }

    }

    intersectObject(object, recursive = false, intersects = []) {

        intersectObject(object, this, intersects, recursive);

        intersects.sort(ascSort);

        return intersects;

    }

    intersectObjects(objects, recursive = false, intersects = []) {

        for (let i = 0, l = objects.length; i < l; i++) {

            intersectObject(objects[i], this, intersects, recursive);

        }

        intersects.sort(ascSort);

        return intersects;

    }



    intersectOctreeObject(faceIndex, nameIndex, instanceMatrixIndex) {
        const intersects = [];

        const [_vA, _vB, _vC] = FaceHelper.getVertices(faceIndex, instanceMatrixIndex)

        let _intersect = this.ray.intersectTriangle(_vA, _vB, _vC, true, _intersectionPoint);


        if (_intersect) {
            _intersectionPointWorld.copy(_intersectionPoint);

            let name = FaceHelper.getName(nameIndex)

            const distance = this.ray.origin.distanceTo(_intersectionPointWorld);

            if (distance > this.near && distance < this.far) {
                let intersection = {
                    distance: distance,
                    // face: { materialIndex: object.materialIndex },
                    point: _intersectionPointWorld.clone(),
                    name: name,
                };
                intersects.push(intersection);
            }

        }


        return intersects;

    }

    intersectOctreeObjects(objects, recursive) {
        // objects是面数据[Int32Array],里面三个一组数据，
        var i, il,
            intersects = [];

        for (i = 0, il = objects.length; i < il; i++) {
            let typeArray = objects[i];

            for (let i = 0; i < typeArray.length; i += 3) {
                const [faceIndex, nameIndex, instanceMatrixIndex] = [typeArray[i], typeArray[i + 1], typeArray[i + 2]]
                intersects = intersects.concat(this.intersectOctreeObject(faceIndex, nameIndex, instanceMatrixIndex));
            }
        }

        // we should sort it, the elements in it aren't arranged by distance.
        intersects.sort(function (a, b) {
            return a.distance - b.distance;
        });

        return intersects

    }

}

function ascSort(a, b) {

    return a.distance - b.distance;

}

function intersectObject(object, raycaster, intersects, recursive) {

    if (object.layers.test(raycaster.layers)) {

        object.raycast(raycaster, intersects);

    }

    if (recursive === true) {

        const children = object.children;

        for (let i = 0, l = children.length; i < l; i++) {

            intersectObject(children[i], raycaster, intersects, true);

        }

    }

}

// #endregion THREE

// #region worker
let octree;

onerror = function (err) {
    console.log(err)
}

onmessage = (e) => {
    const data = e.data;

    switch (data.cmd) {
        case 'init':
            init(data.params);
            break;
        case 'append':
            append(data.params);
            break;

        case 'search':
            search(data.params);
            break;
    }
}


function init(data) {
    data.options.position = new Vector3().fromArray(data.options.position);
    octree = new Octree(data.options);

    data.objects.forEach(meshData => {
        octree.add(meshData);
    });

    octree.update();

    self.postMessage("success init");

}

function append(data) {
    // const boundingBox = new Box3(new Vector3().fromArray(data.options.min), new Vector3().fromArray(data.options.max));
    octree.append(data.objects);
    self.postMessage("success append");
}



function search(data) {
    if (!octree) return

    // 八叉树面对象数据

    const octreeObjects = octree.search(data.origin, data.far, true, data.direction);

    const raycaster = new Raycaster(new Vector3(data.origin.x, data.origin.y, data.origin.z), new Vector3(data.direction.x, data.direction.y, data.direction.z), data.near, data.far);

    const intersections = raycaster.intersectOctreeObjects(octreeObjects);

    self.postMessage(intersections);
}

// #endregion worker


// #region octree

// =========================全局数据========Buffer==========================
const VerticesSize = 300000;
const globeVertices = [new Float32Array(VerticesSize)];  //array  3
let curVecIndex = 0;  //第n个Float32Array
let vecOffset = 0;    //总offset

const MatricesSize = 320000;
const globeMatrices = [new Float32Array(MatricesSize)];  // array 16  matrix4  
let curM4Index = 0;
let m4Offset = 0;

const globeNames = [];    //string  



const FacesSize = 300000
const globeFaces = [new Uint32Array(FacesSize)];   //[2,3,4,  3,4,5] 连续三个是在顶点的序号
let curFaceIndex = 0;
let faceOffset = 0;



class Octree {
    constructor(parameters) {
        parameters = parameters || {};

        parameters.tree = this;

        this.nodeCount = 0;

        this.INDEX_INSIDE_CROSS = - 1;
        this.INDEX_OUTSIDE_OFFSET = 2;

        this.INDEX_OUTSIDE_POS_X = isNumber(parameters.INDEX_OUTSIDE_POS_X) ? parameters.INDEX_OUTSIDE_POS_X : 0;
        this.INDEX_OUTSIDE_NEG_X = isNumber(parameters.INDEX_OUTSIDE_NEG_X) ? parameters.INDEX_OUTSIDE_NEG_X : 1;
        this.INDEX_OUTSIDE_POS_Y = isNumber(parameters.INDEX_OUTSIDE_POS_Y) ? parameters.INDEX_OUTSIDE_POS_Y : 2;
        this.INDEX_OUTSIDE_NEG_Y = isNumber(parameters.INDEX_OUTSIDE_NEG_Y) ? parameters.INDEX_OUTSIDE_NEG_Y : 3;
        this.INDEX_OUTSIDE_POS_Z = isNumber(parameters.INDEX_OUTSIDE_POS_Z) ? parameters.INDEX_OUTSIDE_POS_Z : 4;
        this.INDEX_OUTSIDE_NEG_Z = isNumber(parameters.INDEX_OUTSIDE_NEG_Z) ? parameters.INDEX_OUTSIDE_NEG_Z : 5;

        this.INDEX_OUTSIDE_MAP = [];
        this.INDEX_OUTSIDE_MAP[this.INDEX_OUTSIDE_POS_X] = { index: this.INDEX_OUTSIDE_POS_X, count: 0, x: 1, y: 0, z: 0 };
        this.INDEX_OUTSIDE_MAP[this.INDEX_OUTSIDE_NEG_X] = { index: this.INDEX_OUTSIDE_NEG_X, count: 0, x: - 1, y: 0, z: 0 };
        this.INDEX_OUTSIDE_MAP[this.INDEX_OUTSIDE_POS_Y] = { index: this.INDEX_OUTSIDE_POS_Y, count: 0, x: 0, y: 1, z: 0 };
        this.INDEX_OUTSIDE_MAP[this.INDEX_OUTSIDE_NEG_Y] = { index: this.INDEX_OUTSIDE_NEG_Y, count: 0, x: 0, y: - 1, z: 0 };
        this.INDEX_OUTSIDE_MAP[this.INDEX_OUTSIDE_POS_Z] = { index: this.INDEX_OUTSIDE_POS_Z, count: 0, x: 0, y: 0, z: 1 };
        this.INDEX_OUTSIDE_MAP[this.INDEX_OUTSIDE_NEG_Z] = { index: this.INDEX_OUTSIDE_NEG_Z, count: 0, x: 0, y: 0, z: - 1 };

        this.FLAG_POS_X = 1 << (this.INDEX_OUTSIDE_POS_X + 1);
        this.FLAG_NEG_X = 1 << (this.INDEX_OUTSIDE_NEG_X + 1);
        this.FLAG_POS_Y = 1 << (this.INDEX_OUTSIDE_POS_Y + 1);
        this.FLAG_NEG_Y = 1 << (this.INDEX_OUTSIDE_NEG_Y + 1);
        this.FLAG_POS_Z = 1 << (this.INDEX_OUTSIDE_POS_Z + 1);
        this.FLAG_NEG_Z = 1 << (this.INDEX_OUTSIDE_NEG_Z + 1);

        this.utilVec31Search = new Vector3();
        this.utilVec32Search = new Vector3();

        // properties
        this.objectsData = [];

        //  node边长>50就继续往下分
        this.depthMax = isNumber(parameters.depthMax) ? parameters.depthMax : Math.ceil(Math.log2(parameters.radius / 50)) + 2;

        this.objectsThreshold = isNumber(parameters.objectsThreshold) ? parameters.objectsThreshold : 8;

        this.scene = parameters.scene;

        if (this.scene) {

            var helper = new THREE.BoxHelper(new THREE.Mesh(new THREE.BoxBufferGeometry(1, 1, 1)), 0xff0066);
            this.visualGeometry = helper.geometry;
            this.visualMaterial = helper.material;

        }


        // 重叠
        this.overlapPct = isNumber(parameters.overlapPct) ? parameters.overlapPct : 0;
        // 边长放大10%建root
        parameters.radius *= 1.1;
        this.root = new OctreeNode(parameters);

        this.leafNodes = [this.root];

    }

    add(object) {
        // 顶点数组起始
        const vertexStart = vecOffset / 3;

        // 顶点数据对象：包含face等信息的
        const verticesData = [];
        const facesData = []  //

        const _positions = new BufferAttribute(object.positions, 3).applyMatrix4(object.matrixWorld).array;

        for (let i = 0; i < _positions.length; i += 3) {
            if (vecOffset >= VerticesSize * (curVecIndex + 1)) {
                globeVertices.push(new Float32Array(VerticesSize));
                curVecIndex++;
            }

            globeVertices[curVecIndex].set([_positions[i], _positions[i + 1], _positions[i + 2]], vecOffset % VerticesSize)
            vecOffset += 3

            let vertextData = new OctreeObjectData("useVertex")
            vertextData.vertexIndex = vecOffset / 3 - 1

            verticesData.push(vertextData);

        }

        // ======获取每个顶点所在的面的信息，存到顶点对象里
        const indices = object.indices ? object.indices : range(0, _positions.length);

        for (let i = 0; i < indices.length; i += 3) {
            let a = indices[i], b = indices[i + 1], c = indices[i + 2]; //在这个构件中的顶点序号

            if (faceOffset >= FacesSize * (curFaceIndex + 1)) {
                globeFaces.push(new Uint32Array(FacesSize));
                curFaceIndex++;
            }
            globeFaces[curFaceIndex].set([a + vertexStart, b + vertexStart, c + vertexStart], faceOffset % FacesSize)
            faceOffset += 3

            // facesData放的是在golbeFaces中的index
            facesData.push(faceOffset / 3 - 1)


            verticesData[a].facesData.push(facesData.length - 1);
            verticesData[b].facesData.push(facesData.length - 1);
            verticesData[c].facesData.push(facesData.length - 1);

        }


        // =======================到这一步，verticesData里只有vector3的序号，（face是顶点序号）面序号

        switch (object.type) {
            case "isPrimitive":
                globeNames.push(object.name);

                let options = {
                    type: "useInstance",
                    verticesData: verticesData,
                    facesData: facesData,
                    nameIndex: globeNames.length - 1,
                    boundingBox: new Box3().set(object.boundingBoxArray[0].min, object.boundingBoxArray[0].max),
                }

                this.addObjectData(options);

                break;
            case "isInstancedPrimitive":
                const matrixStart = m4Offset / 16;
                if (m4Offset >= MatricesSize * (curM4Index + 1)) {
                    globeMatrices.push(new Float32Array(MatricesSize));
                    curM4Index++;
                }
                let remainingLength = MatricesSize - m4Offset % MatricesSize;
                if (remainingLength >= object.instanceMatrix.array.length) {
                    globeMatrices[curM4Index].set(object.instanceMatrix.array, m4Offset % MatricesSize)
                    m4Offset += object.instanceMatrix.array.length;
                } else {
                    //分段放进去
                    // 先补齐remain的
                    let curArr = Array.from(object.instanceMatrix.array);

                    let remainArr = curArr.splice(remainingLength);

                    globeMatrices[curM4Index].set(curArr, m4Offset % MatricesSize)

                    m4Offset += curArr.length;

                    while (remainArr.length) {
                        globeMatrices.push(new Float32Array(MatricesSize));
                        curM4Index++;

                        curArr = remainArr;
                        remainArr = curArr.splice(MatricesSize);

                        globeMatrices[curM4Index].set(curArr, m4Offset % MatricesSize)
                        m4Offset += curArr.length;

                    }

                }

                for (let i = 0, l = object.count; i < l; i++) {
                    globeNames.push(object.instanceNames[i]);

                    let options = {
                        type: "useInstance",
                        verticesData: verticesData, //里面的face是在facesData里的序号
                        facesData: facesData,
                        nameIndex: globeNames.length - 1,
                        boundingBox: new Box3().set(object.boundingBoxArray[i].min, object.boundingBoxArray[i].max),

                        instanceMatrix: matrixStart + i,
                    }

                    this.addObjectData(options);

                }
                break;
            case "isCombineMesh":
                for (let i = 0, l = object.count; i < l; i++) {
                    const name = object._instanceIdMap.get(i);
                    globeNames.push(name);


                    const groups = object._attributesGroup.map((v, index) => ({
                        start: v.start,
                        count: v.count,
                        index: index,
                        name: v.name,
                        materialIndex: v.materialIndex
                    })).filter(v => v.name === name);

                    let vertices = [];
                    let vertexStart, vertexCount, index, materialIndex;
                    for (let j = 0; j < groups.length; j++) {
                        vertexStart = groups[j].start / 3;
                        vertexCount = groups[j].count;
                        index = groups[j].index;
                        materialIndex = groups[j].materialIndex;

                        let _vertices = verticesData.slice(vertexStart, vertexStart + vertexCount);

                        vertices.push(_vertices);

                    }

                    let options = {
                        type: "useInstance",
                        verticesData: vertices.flat(),
                        facesData: facesData,  //存放了整个combineMesh的面，为了序号正常，因此材质需要在分点时去顶点再赋值
                        nameIndex: globeNames.length - 1,
                        boundingBox: new Box3().set(object.boundingBoxArray[i].min, object.boundingBoxArray[i].max),

                    }

                    this.addObjectData(options);
                }
        }
    }




    addObjectData(options) {
        // 构件对象
        var objectData = new OctreeObjectData(options.type);
        objectData.setInstanceAttributes(options);
        // 构件信息
        this.objectsData.push(objectData);

        this.root.addObject(objectData);

    }

    /**
     *  按instance建树之后，遍历每个构件的顶点，将顶点数据放置到对应叶子节点中，随后删除空节点
     *  只需在add完所有构件数据之后调用一次
     * @memberof Octree
     */
    update() {
        while (this.objectsData.length) {
            const objectData = this.objectsData.shift();
            this.divideVertices(objectData);

        }
        this.removeEmptyNode();

    }

    // 通过判断node包围盒是否包含顶点，将所有顶点数据放置对应叶子节点中
    divideVertices(object) {
        const nodes = object.nodes;

        // 从object.facesData里取面，
        object.verticesData.forEach(v => {

            let point = object.getVertex(v.vertexIndex);  //vector3

            for (let i = 0; i < nodes.length; i++) {
                if (nodes[i].box.containsPoint(point)) {
                    const _facesData = Array.from(new Set(v.facesData))

                    _facesData.forEach(f => {

                        if (nodes[i].offset >= OctreeNode.faceSize * (nodes[i].curFaceIndex + 1)) {
                            nodes[i].facesData.push(new Int32Array(OctreeNode.faceSize));
                            nodes[i].curFaceIndex++;
                        }

                        let instanceMatrixIndex = object.instanceMatrix ? object.instanceMatrix : -1;

                        nodes[i].facesData[nodes[i].curFaceIndex].set([object.facesData[f], object.nameIndex, instanceMatrixIndex], nodes[i].offset % OctreeNode.faceSize);
                        nodes[i].offset += 3


                    })
                    break;
                }
            }

        })

    }

    // 获取所有的node
    getAllChildFromNode(node, nodes) {
        if (!node.nodesIndices.length) return;
        const _nodes = node.nodesIndices.map(i => node.nodesByIndex[i]);
        nodes.push(..._nodes);
        _nodes.forEach(n => this.getAllChildFromNode(n, nodes));
    }

    // 移除没有顶点的node,root没参与
    removeEmptyNode() {
        const nodes = [];
        this.getAllChildFromNode(this.root, nodes);

        let leafNodes = nodes.filter(n => n.depth == this.depthMax - 1).filter(n => n.offset);

        if (leafNodes.length) {
            this.leafNodes = leafNodes;
        }

        const queue = [...this.leafNodes], target = [];
        while (queue.length) {
            const _ = queue.shift();
            target.push(_);
            if (_.parent) {
                queue.push(_.parent);
            }
        }


        nodes.filter(n => !target.includes(n)).forEach(this.removeNode.bind(this));
    }

    removeNode(node) {
        const indexOctant = node.indexOctant;
        const parent = node.parent;
        // 删父节点的nodesIndices
        const index = indexOfValue(parent.nodesIndices, indexOctant);

        parent.nodesIndices.splice(index, 1);

        // 删nodesByIndex
        delete parent.nodesByIndex[indexOctant];

        if (this.scene) {
            this.scene.remove(node.visual);
        }

        node.setParent(undefined);
        this.nodeCount--;
    }

    append(objectsData) {
        objectsData.forEach(data => this.add(data));
        this.update();
    }



    search(position, radius, organizeByObject, direction) {
        // ensure radius (i.e. distance of ray) is a number
        if (!(radius > 0)) {
            radius = Number.MAX_VALUE;
        }

        // if direction passed, normalize and find pct
        let directionPct;
        if (direction instanceof Vector3) {
            direction = this.utilVec31Search.copy(direction).normalize();
            directionPct = this.utilVec32Search.set(1, 1, 1).divide(direction);
        }



        return this.leafNodes.flatMap(node => node.search(position, radius, direction, directionPct))

    }

}

const _constantArray8 = range(0, 8);

class OctreeNode {
    static faceSize = 3000
    constructor(parameters) {
        this.facesData = [new Int32Array(OctreeNode.faceSize)]
        this.curFaceIndex = 0;
        this.offset = 0;

        // utility
        this.utilVec31Branch = new Vector3();
        this.utilVec31Expand = new Vector3();
        this.utilVec31Ray = new Vector3();

        // handle parameters
        parameters = parameters || {};

        // store or create tree
        if (parameters.tree instanceof Octree) {

            this.tree = parameters.tree;

        } else if (parameters.parent instanceof OctreeNode !== true) {

            parameters.root = this;

            this.tree = new Octree(parameters);

        }

        // basic properties
        this.id = this.tree.nodeCount++;
        this.position = parameters.position.isVector3 ? parameters.position : new Vector3();

        this.radius = parameters.radius > 0 ? parameters.radius : 1;

        this.indexOctant = parameters.indexOctant;
        this.depth = 0;

        // reset and assign parent
        this.reset();
        this.setParent(parameters.parent);

        this._index = 0;


        // additional properties
        //  overlapPct   重叠率
        this.overlap = this.radius * this.tree.overlapPct;
        this.radiusOverlap = this.radius + this.overlap;
        this.left = this.position.x - this.radiusOverlap;
        this.right = this.position.x + this.radiusOverlap;
        this.bottom = this.position.y - this.radiusOverlap;
        this.top = this.position.y + this.radiusOverlap;
        this.back = this.position.z - this.radiusOverlap;
        this.front = this.position.z + this.radiusOverlap;


        // 节点包围盒
        const min = new Vector3(this.left, this.bottom, this.back);
        const max = new Vector3(this.right, this.top, this.front);
        this.box = new Box3(min, max);


        // visual
        if (this.tree.scene) {

            this.visual = new THREE.LineSegments(this.tree.visualGeometry, this.tree.visualMaterial);
            this.visual.scale.set(this.radiusOverlap * 2, this.radiusOverlap * 2, this.radiusOverlap * 2);
            this.visual.position.copy(this.position);

            this.tree.scene.add(this.visual);

        }

    }
    addObject(object) {
        let nodes = [this];
        let depth = this.depth;
        while (depth < this.tree.depthMax - 1) {

            let n = nodes.flatMap(n => _constantArray8.map(i => n.branch(i)))

            nodes = n.filter(n => n.box.intersectsBox(object.boundingBox));

            depth += 1;

        }

        object.nodes.push(...nodes);

    }
    search(position, radius, direction, directionPct) {

        let intersects;

        // test intersects by parameters

        if (direction) {
            // 这个node有没有跟这个射线相交
            intersects = this.intersectRay(position, direction, radius, directionPct);

        } else {

            intersects = this.intersectSphere(position, radius);

        }


        // if intersects

        if (intersects === true) {

            return this.facesData;

        }


        return [];

    }

    reset(cascade, removeVisual) {

        var i, l,
            node,
            nodesIndices = this.nodesIndices || [],
            nodesByIndex = this.nodesByIndex;

        this.objects = [];
        this.nodesIndices = [];
        this.nodesByIndex = {};

        // unset parent in nodes

        for (i = 0, l = nodesIndices.length; i < l; i++) {

            node = nodesByIndex[nodesIndices[i]];

            node.setParent(undefined);

            if (cascade === true) {

                node.reset(cascade, removeVisual);

            }

        }

        // visual

        if (removeVisual === true && this.visual && this.visual.parent) {

            this.visual.parent.remove(this.visual);

        }

    }


    setParent(parent) {

        // store new parent

        if (parent !== this && this.parent !== parent) {

            this.parent = parent;

            this.updateProperties();

        }

    }

    updateProperties() {

        var i, l;

        // properties

        if (this.parent instanceof OctreeNode) {

            this.tree = this.parent.tree;
            this.depth = this.parent.depth + 1;

        } else {

            this.depth = 0;

        }

        // cascade

        for (i = 0, l = this.nodesIndices.length; i < l; i++) {

            this.nodesByIndex[this.nodesIndices[i]].updateProperties();

        }

    }

    branch(indexOctant) {

        var node,
            overlap,
            radius,
            radiusOffset,
            offset,
            position;

        // node exists

        if (this.nodesByIndex[indexOctant] instanceof OctreeNode) {

            node = this.nodesByIndex[indexOctant];

        } else {

            // properties

            radius = (this.radiusOverlap) * 0.5;
            overlap = radius * this.tree.overlapPct;
            radiusOffset = radius - overlap;
            offset = this.utilVec31Branch.set(indexOctant & 1 ? radiusOffset : - radiusOffset, indexOctant & 2 ? radiusOffset : - radiusOffset, indexOctant & 4 ? radiusOffset : - radiusOffset);
            position = new Vector3().addVectors(this.position, offset);



            // node


            node = new OctreeNode({
                tree: this.tree,
                parent: this,
                position: position,
                radius: radius,
                indexOctant: indexOctant,
            });

            // store

            this.addNode(node, indexOctant);

        }

        return node;

    }

    addNode(node, indexOctant) {

        node.indexOctant = indexOctant;

        if (indexOfValue(this.nodesIndices, indexOctant) === - 1) {

            this.nodesIndices.push(indexOctant);

        }

        this.nodesByIndex[indexOctant] = node;

        if (node.parent !== this) {

            node.setParent(this);

        }

    }


    intersectSphere(position, radius) {

        var distance = radius * radius,
            px = position.x,
            py = position.y,
            pz = position.z;

        if (px < this.left) {

            distance -= Math.pow(px - this.left, 2);

        } else if (px > this.right) {

            distance -= Math.pow(px - this.right, 2);

        }

        if (py < this.bottom) {

            distance -= Math.pow(py - this.bottom, 2);

        } else if (py > this.top) {

            distance -= Math.pow(py - this.top, 2);

        }

        if (pz < this.back) {

            distance -= Math.pow(pz - this.back, 2);

        } else if (pz > this.front) {

            distance -= Math.pow(pz - this.front, 2);

        }

        return distance >= 0;

    }

    intersectRay(origin, direction, distance, directionPct) {

        if (typeof directionPct === 'undefined') {

            directionPct = this.utilVec31Ray.set(1, 1, 1).divide(direction);

        }

        var t1 = (this.left - origin.x) * directionPct.x,
            t2 = (this.right - origin.x) * directionPct.x,
            t3 = (this.bottom - origin.y) * directionPct.y,
            t4 = (this.top - origin.y) * directionPct.y,
            t5 = (this.back - origin.z) * directionPct.z,
            t6 = (this.front - origin.z) * directionPct.z,
            tmax = Math.min(Math.min(Math.max(t1, t2), Math.max(t3, t4)), Math.max(t5, t6)),
            tmin;

        // ray would intersect in reverse direction, i.e. this is behind ray
        if (tmax < 0) {

            return false;

        }

        tmin = Math.max(Math.max(Math.min(t1, t2), Math.min(t3, t4)), Math.min(t5, t6));

        // if tmin > tmax or tmin > ray distance, ray doesn't intersect AABB
        if (tmin > tmax || tmin > distance) {

            return false;

        }

        return true;

    }


}


const _m2 = new Matrix4();


class OctreeObjectData {

    constructor(type) {

        switch (type) {
            case "useVertex":
                this.facesData = [];   //在globeFaces中的序号
                break;
            case "useFace":

                break;

            case "useInstance":
                break;

        }

    }

    setInstanceAttributes(options) {

        this.nameIndex = options.nameIndex;
        this.instanceMatrix = options.instanceMatrix

        this.boundingBox = options.boundingBox;
        this.verticesData = options.verticesData;
        this.facesData = options.facesData;


        // 构件属于哪些node   octreeNode
        this.nodes = [];

    }

    getVertex(i) {

        let index = Math.ceil((i + 1) * 3 / VerticesSize) - 1;
        let offset = i * 3 % VerticesSize;
        let vertex = new Vector3(globeVertices[index][offset], globeVertices[index][offset + 1], globeVertices[index][offset + 2]);


        if (this.instanceMatrix !== undefined) {
            let index_m2 = Math.ceil((this.instanceMatrix + 1) * 16 / MatricesSize) - 1;
            let offset_m2 = this.instanceMatrix * 16 % MatricesSize;

            _m2.fromArray(globeMatrices[index_m2], offset_m2)

            vertex.applyMatrix4(_m2)
        }

        return vertex;

    }


    getVertexIndices(i) {
        let index = Math.ceil((i + 1) * 3 / FacesSize) - 1;
        let offset = i * 3 % FacesSize;
        return [globeFaces[index][offset], globeFaces[index][offset + 1], globeFaces[index][offset + 2]];
    }


    // 返回string
    getName() {
        return globeNames[this.nameIndex]
    }

    // 不同构件放不同的OctreeObjectData
    clone(type) {
        const data = new OctreeObjectData(type);
        for (const key in this) {
            data[key] = this[key]
        }

        return data

    }


};





// 帮助类
class FaceHelper {
    // 获取面的三个顶点的vector3
    static getVertices(faceIndex, instanceMatrixIndex) {
        let index = Math.ceil((faceIndex + 1) * 3 / FacesSize) - 1; //第index个typeArray里
        let offset = faceIndex * 3 % FacesSize;  //第offset开始

        return [FaceHelper._getVertex(globeFaces[index][offset], instanceMatrixIndex), FaceHelper._getVertex(globeFaces[index][offset + 1], instanceMatrixIndex), FaceHelper._getVertex(globeFaces[index][offset + 2], instanceMatrixIndex)];
    }


    static getName(nameIndex) {
        return globeNames[nameIndex]
    }

    // 获取globeVertices中第i个顶点
    static _getVertex(vertexIndex, instanceMatrixIndex) {
        let index = Math.ceil((vertexIndex + 1) * 3 / VerticesSize) - 1;
        let offset = vertexIndex * 3 % VerticesSize;
        let vertex = new Vector3(globeVertices[index][offset], globeVertices[index][offset + 1], globeVertices[index][offset + 2]);


        if (instanceMatrixIndex !== -1) {
            let index_m2 = Math.ceil((instanceMatrixIndex + 1) * 16 / MatricesSize) - 1;
            let offset_m2 = instanceMatrixIndex * 16 % MatricesSize;

            _m2.fromArray(globeMatrices[index_m2], offset_m2)

            vertex.applyMatrix4(_m2)
        }

        return vertex;

    }


}

function isNumber(n) {

    return !isNaN(n) && isFinite(n);

}


function indexOfValue(array, value) {

    for (var i = 0, il = array.length; i < il; i++) {

        if (array[i] === value) {
            return i;
        }
    }
    return - 1;

}

// 返回一个从start到end的数组 eg:start=0,end=3,array = [0,1,2]
function range(start, end) {
    const array = [];
    for (let i = start; i < end; i++) {
        array.push(i);
    }
    return array;
}

// #endregion octree




        `;

        const url = window.URL.createObjectURL(new Blob([workerContent], { type: 'application/javascript' }));

        this.pool.setWorkerCreator(() => new Worker(url));
    }

    private _processData(meshArray: (CombineMesh | Primitive | InstancedPrimitive)[]) {
        return meshArray.map((obj) => {
            // 缓存name对应的mesh
            if (isPrimitive(obj)) {
                this._primitiveCache[obj.name] = obj;
            } else {
                obj.instanceNames.forEach((name) => (this._primitiveCache[name] = obj));
            }

            let materialSideByIndex: { [key: number]: number } = {};

            const materials = Array.isArray(obj.material) ? obj.material : [obj.material];

            materials.forEach((material, index) => {
                materialSideByIndex[index] = material.side;
            });

            if (isPrimitive(obj)) {
                return {
                    type: 'isPrimitive',
                    positions: obj.geometry.attributes.position.array,
                    indices: obj.geometry.index?.array,
                    name: obj.name,
                    matrixWorld: obj.matrixWorld,
                    boundingBoxArray: [obj.boundingBox],
                    materialSideByIndex: materialSideByIndex, //材质数组对应的side{0：0，1：0}
                };
            } else if (isInstancedPrimitive(obj)) {
                return {
                    type: 'isInstancedPrimitive',
                    positions: obj.geometry.attributes.position.array,
                    indices: obj.geometry.index?.array,
                    instanceNames: obj.instanceNames,
                    instanceMatrix: obj.instanceMatrix,
                    count: obj.count,
                    matrixWorld: obj.matrixWorld,
                    boundingBoxArray: Array.from(new Array(obj.count).keys()).map((i) => obj.getBoundingBoxAt(i)),
                    materialSideByIndex: materialSideByIndex, //材质数组对应的side{0：0，1：0}
                };
            }

            if (isCombineMesh(obj) && obj.isReady) {
                return {
                    type: 'isCombineMesh',
                    positions: obj.positions,
                    indices: obj.indices,
                    _attributesGroup: obj.attributesGroup,
                    _instanceIdMap: obj.instanceIdMap,
                    count: obj.count,
                    matrixWorld: obj.matrixWorld,
                    boundingBoxArray: Array.from(new Array(obj.count).keys()).map((i) => obj.getBoundingBoxAt(i)),
                    materialSideByIndex: materialSideByIndex, //材质数组对应的side{0：0，1：0}
                };
            }
        });
    }

    init(objects: CombineMesh[], options: IOctree) {
        console.time('init');
        const objectsData = this._processData(objects);

        options.position = isVector3(options.position) ? options.position.toArray() : options.position;
        const initParams = {
            options: options,
            objects: objectsData,
        };

        this.pool
            .postMessage({
                cmd: 'init',
                params: initParams,
            })
            .then((data) => {
                this.isReady = true;
                console.timeEnd('init');
            });
    }

    append(objects: CombineMesh[], boundingBox: Box3) {
        console.time('append');
        this.isReady = false;
        const objectsData = this._processData(objects);
        const appendParams = {
            options: { max: boundingBox.max.toArray(), min: boundingBox.min.toArray() },
            objects: objectsData,
        };

        this.pool
            .postMessage({
                cmd: 'append',
                params: appendParams,
            })
            .then((data) => {
                this.isReady = true;

                console.timeEnd('append');
            });
    }

    search(raycaster: Raycaster): Promise<Intersection[]> {
        return this.pool
            .postMessage({
                cmd: 'search',
                params: {
                    origin: raycaster.ray.origin,
                    direction: raycaster.ray.direction,
                    near: raycaster.near,
                    far: raycaster.far,
                },
            })
            .then((data) => {
                const intersections = (data.data as (Intersection & { name: string })[]).map((i) => {
                    i.object = this._primitiveCache[i.name];
                    i.point = new Vector3(i.point.x, i.point.y, i.point.z);
                    // i.instanceId = (i.object as CombineMesh).getInstanceIdByName(i.name);
                    return i;
                });
                return intersections;
            });
    }
}
