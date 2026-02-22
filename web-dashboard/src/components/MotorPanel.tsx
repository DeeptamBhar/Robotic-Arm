import React from 'react';
import { RotateCcw } from 'lucide-react';

interface MotorData {
    id: number;
    velocity: number;
    acceleration: number;
    angle: number;
}

interface MotorPanelProps {
    data: MotorData;
    onReset: (id: number) => void;
    onAngleChange?: (id: number, angle: number) => void;
}

export const MotorPanel: React.FC<MotorPanelProps> = ({ data, onReset, onAngleChange }) => {
    // Threshold limits as per python code
    const VEL_LIMIT = 8.0;
    const ACC_LIMIT = 6.0;
    const ANG_LIMIT = 170.0;

    const isOk =
        Math.abs(data.velocity) <= VEL_LIMIT &&
        Math.abs(data.acceleration) <= ACC_LIMIT &&
        data.angle >= 0 && data.angle <= ANG_LIMIT;

    const isWarning =
        !isOk && (
            Math.abs(data.velocity) <= VEL_LIMIT * 1.2 &&
            Math.abs(data.acceleration) <= ACC_LIMIT * 1.2 &&
            data.angle >= -10 && data.angle <= ANG_LIMIT + 10
        );

    const statusClass = isOk ? 'status-ok' : isWarning ? 'status-warning' : 'status-error';

    return (
        <div className={`glass-panel motor-panel ${statusClass}`}>
            <div className="motor-header">
                <h3 className="motor-title">Motor {data.id}</h3>
                {isOk ? (
                    <span style={{ color: 'var(--accent-green)', fontSize: '0.875rem', fontWeight: 500 }}>Nominal</span>
                ) : isWarning ? (
                    <span style={{ color: '#f59e0b', fontSize: '0.875rem', fontWeight: 500 }}>Warning</span>
                ) : (
                    <span style={{ color: 'var(--accent-red)', fontSize: '0.875rem', fontWeight: 500 }}>Critical</span>
                )}
            </div>

            <div className="metric-group">
                <div className="metric-item">
                    <span className="metric-label">Velocity</span>
                    <span className="metric-value">
                        {data.velocity.toFixed(2)} <span className="metric-unit">rad/s</span>
                    </span>
                    <div className="value-bar-container">
                        <div
                            className="value-bar"
                            style={{
                                width: `${Math.min(100, (Math.abs(data.velocity) / (VEL_LIMIT * 1.5)) * 100)}%`,
                                backgroundColor: Math.abs(data.velocity) <= VEL_LIMIT ? 'var(--accent-green)' : 'var(--accent-red)'
                            }}
                        />
                    </div>
                </div>

                <div className="metric-item">
                    <span className="metric-label">Acceleration</span>
                    <span className="metric-value">
                        {data.acceleration.toFixed(2)} <span className="metric-unit">rad/s²</span>
                    </span>
                    <div className="value-bar-container">
                        <div
                            className="value-bar"
                            style={{
                                width: `${Math.min(100, (Math.abs(data.acceleration) / (ACC_LIMIT * 1.5)) * 100)}%`,
                                backgroundColor: Math.abs(data.acceleration) <= ACC_LIMIT ? 'var(--accent-blue)' : '#f59e0b'
                            }}
                        />
                    </div>
                </div>

                <div className="metric-item">
                    <span className="metric-label">Angle</span>
                    <span className="metric-value">
                        {data.angle.toFixed(2)}°
                    </span>
                    <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem', marginTop: '0.25rem' }}>
                        <div className="value-bar-container" style={{ margin: 0 }}>
                            <div
                                className="value-bar"
                                style={{
                                    width: `${Math.min(100, (Math.max(0, data.angle) / ANG_LIMIT) * 100)}%`,
                                    backgroundColor: (data.angle >= 0 && data.angle <= ANG_LIMIT) ? '#fff' : 'var(--accent-red)'
                                }}
                            />
                        </div>
                        {onAngleChange && (
                            <input
                                type="range"
                                min="-180"
                                max="180"
                                value={data.angle}
                                onChange={(e) => onAngleChange(data.id, parseFloat(e.target.value))}
                                style={{ width: '100%', cursor: 'pointer' }}
                            />
                        )}
                    </div>
                </div>
            </div>

            <button className="motor-btn" onClick={() => onReset(data.id)}>
                <RotateCcw size={16} />
                Reset Motor
            </button>
        </div>
    );
};
