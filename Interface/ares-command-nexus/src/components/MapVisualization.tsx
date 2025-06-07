import React, { useState, useEffect, useRef } from 'react';
import { Card, CardContent, CardFooter, CardHeader, CardTitle } from '@/components/ui/card';
import { Badge } from '@/components/ui/badge';

const CELL_SIZE = 5;

const MapVisualization: React.FC = () => {
  const [mapData, setMapData] = useState<number[][]>([]);
  const [robotPosition, setRobotPosition] = useState({ x: 0, y: 0 });
  const [mapOrigin, setMapOrigin] = useState({ x: 0, y: 0 });
  const [mapResolution, setMapResolution] = useState(0.05);
  const [offset, setOffset] = useState({ x: 0, y: 0 });
  const [zoom, setZoom] = useState(1);
  const [followBot, setFollowBot] = useState(true);
  const [robotHeading, setRobotHeading] = useState<number | null>(null);
  const [robotSpeed, setRobotSpeed] = useState<number | null>(null);

  const canvasRef = useRef<HTMLCanvasElement>(null);
  const offsetRef = useRef(offset);

  // Fetch occupancy grid and robot pose
  useEffect(() => {
    const fetchData = async () => {
      try {
        const mapRes = await fetch('http://localhost:5000/occupancy_grid');
        const mapResult = await mapRes.json();
        if (mapResult.status === 'ok') {
          setMapData(mapResult.data);
          setMapOrigin(mapResult.origin.position);
          setMapResolution(mapResult.resolution);
        }

        const poseRes = await fetch('http://localhost:5000/robot_pose');
        const poseResult = await poseRes.json();
        if (poseResult.status === 'ok') {
          const { x, y } = poseResult.position;
          const { linear } = poseResult.velocity;

          setRobotPosition({ x, y });

          const headingRad = Math.atan2(linear.y, linear.x);
          const headingDeg = (headingRad * 180) / Math.PI;
          setRobotHeading(Math.round(headingDeg));

          const speed = Math.hypot(linear.x, linear.y);
          setRobotSpeed(parseFloat(speed.toFixed(2)));
        }
      } catch (error) {
        console.error('Error fetching map or robot pose:', error);
      }
    };

    fetchData();
    const interval = setInterval(fetchData, 1000);
    return () => clearInterval(interval);
  }, []);

  // Center on robot initially and when following
  useEffect(() => {
    if (!canvasRef.current || !mapData.length || !followBot) return;

    const canvas = canvasRef.current;

    const botGridX = (robotPosition.x - mapOrigin.x) / mapResolution;
    const botGridY = (robotPosition.y - mapOrigin.y) / mapResolution;
    const canvasBotX = botGridX * CELL_SIZE * zoom;
    const canvasBotY = (mapData.length - botGridY) * CELL_SIZE * zoom; // flip Y

    const centerOffset = {
      x: canvas.width / 2 - canvasBotX,
      y: canvas.height / 2 - canvasBotY,
    };

    offsetRef.current = centerOffset;
    setOffset(centerOffset);
  }, [robotPosition, zoom, mapData, followBot, mapOrigin, mapResolution]);

  const drawMap = (currentOffset = offsetRef.current) => {
    const canvas = canvasRef.current;
    if (!canvas || !mapData.length) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    const rows = mapData.length;
    const cols = mapData[0].length;

    for (let y = 0; y < rows; y++) {
      for (let x = 0; x < cols; x++) {
        const value = mapData[y][x];
        let color = '#808080';
        if (value === 0) color = '#ffffff';
        else if (value === 100) color = '#000000';

        ctx.fillStyle = color;
        ctx.fillRect(
          currentOffset.x + x * CELL_SIZE * zoom,
          currentOffset.y + y * CELL_SIZE * zoom,
          CELL_SIZE * zoom,
          CELL_SIZE * zoom
        );
      }
    }

    // Draw robot
    const robotGridX = (robotPosition.x - mapOrigin.x) / mapResolution;
    const robotGridY = (robotPosition.y - mapOrigin.y) / mapResolution;
    const robotCanvasX = currentOffset.x + robotGridX * CELL_SIZE * zoom + (CELL_SIZE * zoom) / 2;
    const robotCanvasY = currentOffset.y + (mapData.length - robotGridY) * CELL_SIZE * zoom + (CELL_SIZE * zoom) / 2;

    ctx.fillStyle = '#00ff00';
    ctx.beginPath();
    ctx.arc(robotCanvasX, robotCanvasY, CELL_SIZE * zoom, 0, 2 * Math.PI);
    ctx.fill();
  };

  const handleResetZoom = () => {
    setZoom(1);
    setFollowBot(true);

    if (!canvasRef.current || !mapData.length) return;

    const canvas = canvasRef.current;

    const botGridX = (robotPosition.x - mapOrigin.x) / mapResolution;
    const botGridY = (robotPosition.y - mapOrigin.y) / mapResolution;
    const canvasBotX = botGridX * CELL_SIZE;
    const canvasBotY = (mapData.length - botGridY) * CELL_SIZE;

    const centerOffset = {
      x: canvas.width / 2 - canvasBotX,
      y: canvas.height / 2 - canvasBotY,
    };

    offsetRef.current = centerOffset;
    setOffset(centerOffset);
  };

  // ZOOM
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const handleWheel = (e: WheelEvent) => {
      e.preventDefault();

      const zoomIntensity = 0.1;
      const direction = e.deltaY > 0 ? -1 : 1;
      const newZoom = Math.min(Math.max(zoom + direction * zoomIntensity, 0.5), 4); // clamp between 0.5x and 4x

      const rect = canvas.getBoundingClientRect();
      const mouseX = e.clientX - rect.left;
      const mouseY = e.clientY - rect.top;

      const wx = (mouseX - offsetRef.current.x) / (CELL_SIZE * zoom);
      const wy = (mouseY - offsetRef.current.y) / (CELL_SIZE * zoom);

      const newOffset = {
        x: mouseX - wx * CELL_SIZE * newZoom,
        y: mouseY - wy * CELL_SIZE * newZoom
      };

      offsetRef.current = newOffset;
      setOffset(newOffset);
      setZoom(newZoom);
    };

    canvas.addEventListener('wheel', handleWheel, { passive: false });

    return () => {
      canvas.removeEventListener('wheel', handleWheel);
    };
  }, [zoom]);

  useEffect(() => {
    drawMap();
  }, [mapData, zoom, robotPosition, offset, mapOrigin, mapResolution]);

  // Mouse drag panning
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    let isDragging = false;
    let dragStart = { x: 0, y: 0 };
    let startOffset = { x: 0, y: 0 };

    const handleMouseDown = (e: MouseEvent) => {
      isDragging = true;
      setFollowBot(false);
      dragStart = { x: e.clientX, y: e.clientY };
      startOffset = { ...offsetRef.current };
    };

    const handleMouseMove = (e: MouseEvent) => {
      if (!isDragging) return;

      const dx = (e.clientX - dragStart.x);
      const dy = (e.clientY - dragStart.y);

      // Apply scaling by zoom
      const newOffset = {
        x: startOffset.x + dx,
        y: startOffset.y + dy
      };

      offsetRef.current = newOffset;
      drawMap(newOffset);
    };

    const handleMouseUp = () => {
      if (isDragging) {
        setOffset(offsetRef.current);
        isDragging = false;
      }
    };

    canvas.addEventListener('mousedown', handleMouseDown);
    window.addEventListener('mousemove', handleMouseMove);
    window.addEventListener('mouseup', handleMouseUp);

    return () => {
      canvas.removeEventListener('mousedown', handleMouseDown);
      window.removeEventListener('mousemove', handleMouseMove);
      window.removeEventListener('mouseup', handleMouseUp);
    };
  }, []);

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
          <canvas
            ref={canvasRef}
            width={800}
            height={600}
            className="w-full h-full border border-gray-700 rounded"
          />

          <div className="absolute top-2 left-2 text-military-green text-xs font-mono">
            <div>SLAM: ACTIVE</div>
            <div>RES: {mapResolution} m/px</div>
            <div>SIZE: {mapData[0]?.length || 0}x{mapData.length || 0}</div>
          </div>

          <div className="absolute top-2 right-2 text-military-red text-xs font-mono text-right">
            <div>ROBOT: ({robotPosition.x.toFixed(1)}, {robotPosition.y.toFixed(1)})</div>
            <div>HEADING: {robotHeading !== null ? `${robotHeading}°` : 'N/A'}</div>
            <div>SPEED: {robotSpeed !== null ? `${robotSpeed} m/s` : 'N/A'}</div>
          </div>
          <button
            onClick={handleResetZoom}
            className="absolute bottom-2 right-2 bg-military-red text-white px-3 py-1 text-xs rounded shadow m-2"
          >
            Reset Zoom
          </button>

          <div className="absolute bottom-2 left-2 space-y-1">
            <div className="flex items-center space-x-2 text-xs">
              <div className="w-3 h-3 bg-white border border-gray-400" />
              <span className="text-gray-400">Free</span>
            </div>
            <div className="flex items-center space-x-2 text-xs">
              <div className="w-3 h-3 bg-black" />
              <span className="text-gray-400">Occupied</span>
            </div>
            <div className="flex items-center space-x-2 text-xs">
              <div className="w-3 h-3 bg-gray-500" />
              <span className="text-gray-400">Unknown</span>
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
