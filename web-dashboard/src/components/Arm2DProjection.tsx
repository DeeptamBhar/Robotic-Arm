import React, { useMemo } from 'react';

interface Arm2DProjectionProps {
    angles: number[];
}

export const Arm2DProjection: React.FC<Arm2DProjectionProps> = ({ angles }) => {
    const L1 = 150; // Scaled up for SVG viewBox
    const L2 = 120;
    const L3 = 100;

    // Calculate points
    const points = useMemo(() => {
        const t1 = (angles[0] || 0) * (Math.PI / 180);
        const t2 = (angles[1] || 0) * (Math.PI / 180);
        const t3 = (angles[2] || 0) * (Math.PI / 180);

        const x0 = 0, y0 = 0, z0 = 0;

        // Joint 1
        const x1 = L1 * Math.cos(t1);
        const y1 = L1 * Math.sin(t1);
        const z1 = 100;

        // Joint 2
        const x2 = x1 + L2 * Math.cos(t1 + t2);
        const y2 = y1 + L2 * Math.sin(t1 + t2);
        const z2 = z1 + 100;

        // Joint 3
        const x3 = x2 + L3 * Math.cos(t1 + t2 + t3);
        const y3 = y2 + L3 * Math.sin(t1 + t2 + t3);
        const z3 = z2 + 50;

        return {
            side: [
                { x: x0, z: z0 },
                { x: x1, z: z1 },
                { x: x2, z: z2 },
                { x: x3, z: z3 }
            ],
            top: [
                { x: x0, y: y0 },
                { x: x1, y: y1 },
                { x: x2, y: y2 },
                { x: x3, y: y3 }
            ]
        };
    }, [angles]);

    const renderSideView = () => {
        // XZ plane. map to SVG coords: X -> x, Z -> y (inverted)
        const pts = points.side;
        const path = pts.map((p, i) => `${i === 0 ? 'M' : 'L'} ${p.x + 300} ${400 - p.z}`).join(' ');

        return (
            <div className="projection-pane">
                <span className="projection-label">Side View (X-Z)</span>
                <svg className="projection-svg" viewBox="0 0 600 500">
                    <g className="grid-layer">
                        {/* Simple Grid X */}
                        {Array.from({ length: 13 }).map((_, i) => (
                            <line key={`v${i}`} x1={i * 50} y1={0} x2={i * 50} y2={500} className="grid-line" />
                        ))}
                        {/* Simple Grid Y */}
                        {Array.from({ length: 11 }).map((_, i) => (
                            <line key={`h${i}`} x1={0} y1={i * 50} x2={600} y2={i * 50} className="grid-line" />
                        ))}
                        <line x1={0} y1={400} x2={600} y2={400} stroke="#334155" strokeWidth={2} />
                        <line x1={300} y1={0} x2={300} y2={500} stroke="#334155" strokeWidth={2} />
                    </g>

                    <path d={path} className="arm-link" fill="none" />

                    {pts.map((p, i) => (
                        i === 0 ? (
                            <rect key={i} x={p.x + 280} y={400 - p.z} width={40} height={20} className="arm-base" rx={4} />
                        ) : (
                            <circle key={i} cx={p.x + 300} cy={400 - p.z} r={i === 3 ? 8 : 10} className="arm-joint"
                                style={i === 3 ? { stroke: 'var(--accent-red)' } : {}} />
                        )
                    ))}
                </svg>
            </div>
        );
    };

    const renderTopView = () => {
        // XY plane. map to SVG coords: X -> x, Y -> y (inverted)
        const pts = points.top;
        const path = pts.map((p, i) => `${i === 0 ? 'M' : 'L'} ${p.x + 300} ${250 - p.y}`).join(' ');

        return (
            <div className="projection-pane">
                <span className="projection-label">Top View (X-Y)</span>
                <svg className="projection-svg" viewBox="0 0 600 500">
                    <g className="grid-layer">
                        {Array.from({ length: 13 }).map((_, i) => (
                            <line key={`tv${i}`} x1={i * 50} y1={0} x2={i * 50} y2={500} className="grid-line" />
                        ))}
                        {Array.from({ length: 11 }).map((_, i) => (
                            <line key={`th${i}`} x1={0} y1={i * 50} x2={600} y2={i * 50} className="grid-line" />
                        ))}
                        <line x1={0} y1={250} x2={600} y2={250} stroke="#334155" strokeWidth={2} />
                        <line x1={300} y1={0} x2={300} y2={500} stroke="#334155" strokeWidth={2} />
                    </g>

                    <path d={path} className="arm-link" fill="none" />

                    {pts.map((p, i) => (
                        i === 0 ? (
                            <circle key={i} cx={p.x + 300} cy={250 - p.y} r={20} className="arm-base" />
                        ) : (
                            <circle key={i} cx={p.x + 300} cy={250 - p.y} r={i === 3 ? 8 : 10} className="arm-joint"
                                style={i === 3 ? { stroke: 'var(--accent-red)' } : {}} />
                        )
                    ))}
                </svg>
            </div>
        );
    };

    return (
        <div className="glass-panel view-2d-container">
            <div className="projections-grid">
                {renderSideView()}
                {renderTopView()}
            </div>
        </div>
    );
};
