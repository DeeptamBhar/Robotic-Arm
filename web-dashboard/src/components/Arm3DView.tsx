import React from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Environment, ContactShadows, Line, Float } from '@react-three/drei';
import { EffectComposer, Bloom } from '@react-three/postprocessing';
import * as THREE from 'three';

interface Arm3DViewProps {
    angles: number[];
    gripperOpen: boolean;
    onMouseMove?: React.MouseEventHandler<HTMLDivElement>;
}

const LinkCylinder: React.FC<{ start: THREE.Vector3, end: THREE.Vector3, radius: number }> = ({ start, end, radius }) => {
    const vec = new THREE.Vector3().subVectors(end, start);
    const center = new THREE.Vector3().addVectors(start, end).multiplyScalar(0.5);
    const length = vec.length();

    const quaternion = new THREE.Quaternion().setFromUnitVectors(
        new THREE.Vector3(0, 1, 0),
        vec.clone().normalize()
    );

    return (
        <mesh position={center} quaternion={quaternion}>
            <cylinderGeometry args={[radius, radius, length, 32]} />
            <meshPhysicalMaterial
                color="#1e293b"
                metalness={0.8}
                roughness={0.2}
                clearcoat={0.5}
            />
        </mesh>
    );
};

const Gripper: React.FC<{ position: THREE.Vector3, quaternion: THREE.Quaternion, gripperOpen: boolean }> = ({ position, quaternion, gripperOpen }) => {
    const offset = gripperOpen ? 0.08 : 0;

    return (
        <group position={position} quaternion={quaternion}>
            {/* Gripper Base Bracket */}
            <mesh position={[0, 0, 0.05]}>
                <boxGeometry args={[0.4, 0.25, 0.1]} />
                <meshStandardMaterial color="#334155" roughness={0.7} />
            </mesh>

            {/* Left Finger Base */}
            <mesh position={[-0.2 - offset, 0, 0.15]}>
                <boxGeometry args={[0.1, 0.1, 0.3]} />
                <meshStandardMaterial color="#1e293b" roughness={0.8} />
            </mesh>
            {/* Left Finger Tip */}
            <mesh position={[-0.15 - offset, 0, 0.35]} rotation={[0, -Math.PI / 6, 0]}>
                <boxGeometry args={[0.08, 0.1, 0.2]} />
                <meshStandardMaterial color="#0f172a" roughness={0.9} />
            </mesh>

            {/* Right Finger Base */}
            <mesh position={[0.2 + offset, 0, 0.15]}>
                <boxGeometry args={[0.1, 0.1, 0.3]} />
                <meshStandardMaterial color="#1e293b" roughness={0.8} />
            </mesh>
            {/* Right Finger Tip */}
            <mesh position={[0.15 + offset, 0, 0.35]} rotation={[0, Math.PI / 6, 0]}>
                <boxGeometry args={[0.08, 0.1, 0.2]} />
                <meshStandardMaterial color="#0f172a" roughness={0.9} />
            </mesh>
        </group>
    );
};

const RoboticArm: React.FC<{ angles: number[], gripperOpen: boolean }> = ({ angles, gripperOpen }) => {
    // Kinematic lengths for 6 DOF
    const L1 = 1.0;
    const L2 = 2.0;
    const L3 = 1.5;
    const L4 = 0.5;
    const L5 = 0.5;

    // Convert angles to radians
    const t1 = THREE.MathUtils.degToRad((angles[0] || 90) - 90);
    const t2 = THREE.MathUtils.degToRad(-(angles[1] || 90) + 90);
    const t3 = THREE.MathUtils.degToRad((angles[2] || 90) - 90);
    const t4 = THREE.MathUtils.degToRad(angles[3] || 0);
    const t5 = THREE.MathUtils.degToRad(angles[4] || 0);
    const t6 = THREE.MathUtils.degToRad(angles[5] || 0);

    // Forward Kinematics
    const p0 = new THREE.Vector3(0, 0, 0);
    const p1 = new THREE.Vector3(0, L1, 0);

    const arm1Dir = new THREE.Vector3(L2 * Math.sin(t2), L2 * Math.cos(t2), 0);
    arm1Dir.applyAxisAngle(new THREE.Vector3(0, 1, 0), t1);
    const p2 = p1.clone().add(arm1Dir);

    const arm2Dir = new THREE.Vector3(L3 * Math.sin(t2 + t3), L3 * Math.cos(t2 + t3), 0);
    arm2Dir.applyAxisAngle(new THREE.Vector3(0, 1, 0), t1);
    const p3 = p2.clone().add(arm2Dir);

    const wrist1Dir = new THREE.Vector3(L4 * Math.cos(t2 + t3 + Math.PI / 2), L4 * Math.sin(t2 + t3 + Math.PI / 2), 0);
    wrist1Dir.applyAxisAngle(new THREE.Vector3(0, 1, 0), t1 + t4);
    const p4 = p3.clone().add(wrist1Dir);

    const wrist2Dir = new THREE.Vector3(L5 * Math.sin(t2 + t3 + t5), L5 * Math.cos(t2 + t3 + t5), 0);
    wrist2Dir.applyAxisAngle(new THREE.Vector3(0, 1, 0), t1 + t4);
    const p5 = p4.clone().add(wrist2Dir);

    // Points array for the unbroken line and joint rendering
    const points = [p0, p1, p2, p3, p4, p5];

    // Gripper Orientation Setup
    const endEffectorVec = new THREE.Vector3().subVectors(p5, p4);
    if (endEffectorVec.lengthSq() < 0.0001) endEffectorVec.set(0, 1, 0);

    const upVector = new THREE.Vector3(0, 1, 0).applyAxisAngle(new THREE.Vector3(0, 0, 1), t6).applyAxisAngle(new THREE.Vector3(0, 1, 0), t1);
    const rotMat = new THREE.Matrix4().lookAt(new THREE.Vector3(0, 0, 0), endEffectorVec, upVector);
    const gripperQuaternion = new THREE.Quaternion().setFromRotationMatrix(rotMat);

    return (
        <group>
            {/* Base Cylinders */}
            <mesh position={[0, -0.2, 0]}>
                <cylinderGeometry args={[0.6, 0.8, 0.4, 32]} />
                <meshStandardMaterial color="#1e1e24" roughness={0.8} />
            </mesh>

            {/* Links (Line) */}
            <Line
                points={points.map(p => [p.x, p.y, p.z])}
                color="#3b82f6"
                lineWidth={5}
            />

            {/* Volumetric Links using Cylinders between joints */}
            <LinkCylinder start={p0} end={p1} radius={0.15} />
            <LinkCylinder start={p1} end={p2} radius={0.12} />
            <LinkCylinder start={p2} end={p3} radius={0.08} />
            <LinkCylinder start={p3} end={p4} radius={0.06} />
            <LinkCylinder start={p4} end={p5} radius={0.05} />

            {/* Joints */}
            {points.map((p, i) => {
                const isEndEffector = i === 5;
                return (
                    <mesh key={i} position={p}>
                        <sphereGeometry args={[i === 0 ? 0.25 : i === 5 ? 0.12 : 0.18, 32, 32]} />
                        {isEndEffector ? (
                            <meshStandardMaterial
                                color="#b91c1c"
                                metalness={0.6}
                                roughness={0.2}
                                emissive="#7f1d1d"
                                emissiveIntensity={0.3}
                            />
                        ) : (
                            <meshStandardMaterial color="#ffffff" metalness={0.5} roughness={0.1} />
                        )}
                    </mesh>
                );
            })}

            {/* Gripper */}
            <Gripper position={p5} quaternion={gripperQuaternion} gripperOpen={gripperOpen} />
        </group>
    );
};

export const Arm3DView: React.FC<Arm3DViewProps> = ({ angles, gripperOpen, onMouseMove }) => {
    return (
        <div className="glass-panel view-3d-container" onMouseMove={onMouseMove}>
            <div className="view-header">
                <h3 className="view-title">3D Digital Twin</h3>
                <p className="view-subtitle">Live Spatial Orientation</p>
            </div>
            <div className="canvas-wrapper">
                <Canvas camera={{ position: [6, 5, 8], fov: 45 }}>
                    <color attach="background" args={['#050508']} />

                    {/* Lighting */}
                    <ambientLight intensity={0.4} />
                    <directionalLight position={[10, 20, 10]} intensity={1.5} castShadow />
                    <spotLight position={[-10, 10, -10]} intensity={2} color="#3b82f6" angle={0.3} penumbra={1} />
                    <pointLight position={[0, -2, 0]} intensity={1} color="#10b981" distance={10} />

                    <Float speed={2} rotationIntensity={0.1} floatIntensity={0.2}>
                        <RoboticArm angles={angles} gripperOpen={gripperOpen} />
                    </Float>

                    {/* Ground Shadow */}
                    <ContactShadows position={[0, -0.6, 0]} opacity={0.6} scale={20} blur={2.5} far={4} color="#000000" />
                    <Environment preset="studio" />

                    <OrbitControls
                        enableDamping
                        dampingFactor={0.05}
                        maxPolarAngle={Math.PI / 2 - 0.05} // don't go below ground
                        minDistance={3}
                        maxDistance={15}
                    />

                    {/* Grid */}
                    <gridHelper args={[30, 30, '#1e293b', '#0f172a']} position={[0, -0.5, 0]} />

                    <EffectComposer>
                        <Bloom luminanceThreshold={1} mipmapBlur intensity={1.0} />
                    </EffectComposer>
                </Canvas>
            </div>
        </div>
    );
};
