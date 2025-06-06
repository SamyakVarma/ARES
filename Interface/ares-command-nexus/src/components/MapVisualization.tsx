
import React, { useState, useEffect } from 'react';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Badge } from '@/components/ui/badge';

const MapVisualization: React.FC = () => {
  const [mapData, setMapData] = useState<number[][]>([]);
  const [robotPosition, setRobotPosition] = useState({ x: 12, y: 8 });
  const [path, setPath] = useState<{x: number, y: number}[]>([]);

  // Generate simulated occupancy grid
  useEffect(() => {
    const generateMap = () => {
      const width = 25;
      const height = 15;
      const grid: number[][] = [];
      
      for (let y = 0; y < height; y++) {
        const row: number[] = [];
        for (let x = 0; x < width; x++) {
          // 0 = free space, 1 = obstacle, 0.5 = unknown
          let value = 0;
          
          // Add some walls and obstacles
          if (x === 0 || x === width - 1 || y === 0 || y === height - 1) {
            value = 1; // Walls
          } else if (Math.random() < 0.15) {
            value = 1; // Random obstacles
          } else if (Math.random() < 0.1) {
            value = 0.5; // Unknown areas
          }
          
          row.push(value);
        }
        grid.push(row);
      }
      
      setMapData(grid);
    };

    generateMap();
    
    // Simulate robot movement
    const interval = setInterval(() => {
      setRobotPosition(prev => ({
        x: Math.max(1, Math.min(23, prev.x + (Math.random() - 0.5) * 2)),
        y: Math.max(1, Math.min(13, prev.y + (Math.random() - 0.5) * 2))
      }));
      
      // Update path
      setPath(prev => {
        const newPath = [...prev, robotPosition];
        return newPath.slice(-20); // Keep last 20 positions
      });
    }, 2000);

    return () => clearInterval(interval);
  }, [robotPosition]);

  const getCellColor = (value: number) => {
    if (value === 0) return 'bg-gray-800'; // Free space
    if (value === 1) return 'bg-red-600'; // Obstacles
    return 'bg-yellow-600'; // Unknown
  };

  return (
    <Card className="military-panel h-full">
      <CardHeader className="pb-2">
        <div className="flex items-center justify-between">
          <CardTitle className="text-military-red text-sm">2D OCCUPANCY GRID</CardTitle>
          <div className="flex items-center space-x-2">
            <div className="status-indicator bg-military-green" />
            <Badge variant="outline" className="text-xs">MAPPING</Badge>
          </div>
        </div>
      </CardHeader>
      <CardContent className="h-full pb-2">
        <div className="relative h-full bg-black rounded border border-military-border overflow-hidden p-2">
          {/* Grid */}
          <div className="grid grid-cols-25 gap-px h-full" style={{ gridTemplateColumns: 'repeat(25, 1fr)' }}>
            {mapData.map((row, y) =>
              row.map((cell, x) => (
                <div
                  key={`${x}-${y}`}
                  className={`${getCellColor(cell)} relative`}
                >
                  {/* Robot position */}
                  {Math.floor(robotPosition.x) === x && Math.floor(robotPosition.y) === y && (
                    <div className="absolute inset-0 bg-military-green rounded-full animate-pulse" />
                  )}
                  
                  {/* Path trail */}
                  {path.some(p => Math.floor(p.x) === x && Math.floor(p.y) === y) && (
                    <div className="absolute inset-0 bg-blue-500 opacity-50" />
                  )}
                </div>
              ))
            )}
          </div>
          
          {/* Map info overlay */}
          <div className="absolute top-2 left-2 text-military-green text-xs font-mono">
            <div>SLAM: ACTIVE</div>
            <div>RES: 0.05m/px</div>
            <div>SIZE: 25x15</div>
          </div>
          
          <div className="absolute top-2 right-2 text-military-red text-xs font-mono text-right">
            <div>ROBOT: ({robotPosition.x.toFixed(1)}, {robotPosition.y.toFixed(1)})</div>
            <div>HEADING: 45°</div>
            <div>SPEED: 0.8 m/s</div>
          </div>
          
          {/* Legend */}
          <div className="absolute bottom-2 left-2 space-y-1">
            <div className="flex items-center space-x-2 text-xs">
              <div className="w-3 h-3 bg-gray-800 border border-gray-600" />
              <span className="text-gray-400">Free</span>
            </div>
            <div className="flex items-center space-x-2 text-xs">
              <div className="w-3 h-3 bg-red-600" />
              <span className="text-gray-400">Obstacle</span>
            </div>
            <div className="flex items-center space-x-2 text-xs">
              <div className="w-3 h-3 bg-military-green rounded-full" />
              <span className="text-gray-400">Robot</span>
            </div>
          </div>
        </div>
      </CardContent>
    </Card>
  );
};

export default MapVisualization;
