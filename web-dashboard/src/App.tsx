import { useState } from 'react';
import './App.css';
import { MotorPanel } from './components/MotorPanel';
import { Arm3DView } from './components/Arm3DView';
import { Arm2DProjection } from './components/Arm2DProjection';
import { Activity, Grab } from 'lucide-react';

interface MotorData {
  id: number;
  velocity: number;
  acceleration: number;
  angle: number;
}

const generateMockData = (): MotorData[] => {
  return Array.from({ length: 6 }).map((_, i) => ({
    id: i + 1,
    velocity: 0,
    acceleration: 0,
    angle: 0
  }));
};

function App() {
  const [motors, setMotors] = useState<MotorData[]>(() => generateMockData());
  const [gripperOpen, setGripperOpen] = useState(false);

  const handleAngleChange = (id: number, newAngle: number) => {
    setMotors(prev => prev.map(m =>
      m.id === id ? { ...m, angle: newAngle } : m
    ));
  };

  const handleResetMotor = (id: number) => {
    setMotors(prev => prev.map(m =>
      m.id === id ? { ...m, velocity: 0, acceleration: 0, angle: 0 } : m
    ));
  };

  const jointAngles = motors.map(m => m.angle);

  return (
    <div className="app-container">
      <header className="header glass-panel">
        <h1>
          <Activity size={24} color="var(--accent-blue)" />
          Robotic Arm Command Station
        </h1>
        <div style={{ display: 'flex', gap: '1rem', alignItems: 'center' }}>
          <button
            className="motor-btn"
            style={{ margin: 0, padding: '0.5rem 1rem', background: gripperOpen ? 'var(--accent-blue)' : 'var(--bg-tertiary)' }}
            onClick={() => setGripperOpen(!gripperOpen)}
          >
            <Grab size={16} />
            {gripperOpen ? 'Close Gripper' : 'Open Gripper'}
          </button>
          <div className="header-status">
            <div className="status-dot" />
            Live Connection
          </div>
        </div>
      </header>

      <main className="dashboard-grid">
        <section className="motors-grid">
          {motors.map(motor => (
            <MotorPanel
              key={motor.id}
              data={motor}
              onReset={handleResetMotor}
              onAngleChange={handleAngleChange}
            />
          ))}
        </section>

        <section className="visualizations-col">
          <Arm3DView angles={jointAngles} gripperOpen={gripperOpen} />
          <Arm2DProjection angles={jointAngles} />
        </section>
      </main>
    </div>
  );
}

export default App;
