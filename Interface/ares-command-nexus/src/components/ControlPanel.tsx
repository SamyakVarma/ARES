import React, { useState, useRef, useEffect } from 'react';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Switch } from '@/components/ui/switch';
import { Button } from '@/components/ui/button';

interface ControlPanelProps {
  isManualMode: boolean;
  setIsManualMode: (mode: boolean) => void;
}

const sendQuickAction = async (endpoint: string, method = 'POST') => {
  try {
    const res = await fetch(`http://localhost:5000${endpoint}`, { method });
    const data = await res.json();
    console.log(data.message);
  } catch (err) {
    console.error(`Failed to call ${endpoint}`, err);
  }
};

const ControlPanel: React.FC<ControlPanelProps> = ({ isManualMode, setIsManualMode }) => {
  const [joystickPosition, setJoystickPosition] = useState({ x: 0, y: 0 });
  const [isDragging, setIsDragging] = useState(false);
  const [keyboardVelocity, setKeyboardVelocity] = useState({ x: 0, y: 0 });
  const [activeKeys, setActiveKeys] = useState(new Set<string>());
  const [targetSelected, setTargetSelected] = useState(false);

  const [turretPosition, setTurretPosition] = useState({ x: 0, y: 0 });
  const [isTurretDragging, setIsTurretDragging] = useState(false);

  const joystickRef = useRef<HTMLDivElement>(null);
  const turretJoystickRef = useRef<HTMLDivElement>(null);
  const activeKeysRef = useRef(new Set<string>());

  const publishVelocity = (vx: number, vz: number) => {
    fetch('http://localhost:5000/cmd_vel', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ linear_x: vx, angular_z: vz })
    }).catch(err => console.error('Error publishing to /cmd_vel:', err));
  };

  const publishTurretControl = (x: number, y: number) => {
    fetch('http://localhost:5000/turret_cmd', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ aim_x: x, aim_y: y })
    }).catch(err => console.error('Error publishing to /turret_cmd:', err));
  };

  const handleJoystickMove = (clientX: number, clientY: number) => {
    if (!joystickRef.current || !isDragging) return;

    const rect = joystickRef.current.getBoundingClientRect();
    const centerX = rect.left + rect.width / 2;
    const centerY = rect.top + rect.height / 2;

    const deltaX = clientX - centerX;
    const deltaY = clientY - centerY;
    const distance = Math.sqrt(deltaX ** 2 + deltaY ** 2);
    const maxDistance = rect.width / 2 - 16;

    let x, y;
    if (distance <= maxDistance) {
      x = deltaX;
      y = deltaY;
    } else {
      const angle = Math.atan2(deltaY, deltaX);
      x = Math.cos(angle) * maxDistance;
      y = Math.sin(angle) * maxDistance;
    }

    setJoystickPosition({ x, y });

    const vx = +(x / maxDistance * 2).toFixed(2);
    const vz = +(-y / maxDistance * 2).toFixed(2);
    publishVelocity(vx, vz);
  };

  const handleTurretMove = (clientX: number, clientY: number) => {
    if (!turretJoystickRef.current || !isTurretDragging) return;

    const rect = turretJoystickRef.current.getBoundingClientRect();
    const centerX = rect.left + rect.width / 2;
    const centerY = rect.top + rect.height / 2;

    const deltaX = clientX - centerX;
    const deltaY = clientY - centerY;
    const distance = Math.sqrt(deltaX ** 2 + deltaY ** 2);
    const maxDistance = rect.width / 2 - 16;

    let x, y;
    if (distance <= maxDistance) {
      x = deltaX;
      y = deltaY;
    } else {
      const angle = Math.atan2(deltaY, deltaX);
      x = Math.cos(angle) * maxDistance;
      y = Math.sin(angle) * maxDistance;
    }

    setTurretPosition({ x, y });

    const aimX = +(x / maxDistance).toFixed(2);
    const aimY = +(-y / maxDistance).toFixed(2);
    publishTurretControl(aimX, aimY);
  };

  const handleMouseMove = (e: MouseEvent) => {
    if (isDragging) handleJoystickMove(e.clientX, e.clientY);
    if (isTurretDragging) handleTurretMove(e.clientX, e.clientY);
  };

  const handleMouseUp = () => {
    if (isDragging) {
      setIsDragging(false);
      setJoystickPosition({ x: 0, y: 0 });
      if (activeKeysRef.current.size === 0) {
        publishVelocity(0, 0);
      }
    }

    if (isTurretDragging) {
      setIsTurretDragging(false);
      setTurretPosition({ x: 0, y: 0 });
      publishTurretControl(0, 0);
    }
  };

  const handleMouseDown = (e: React.MouseEvent) => {
    if (!isManualMode) return;
    setIsDragging(true);
    handleJoystickMove(e.clientX, e.clientY);
  };

  const updateKeyboardVelocity = () => {
    const keys = activeKeysRef.current;
    let vx = 0, vz = 0;

    if (keys.has('w')) vx += 2;
    if (keys.has('s')) vx -= 2;
    if (keys.has('a')) vz += 2;
    if (keys.has('d')) vz -= 2;

    setKeyboardVelocity({ x: vx, y: vz });
    publishVelocity(vx, vz);
  };

  const handleKeyDown = (e: KeyboardEvent) => {
    if (!isManualMode) return;
    const key = e.key.toLowerCase();
    if (['w', 'a', 's', 'd'].includes(key)) {
      e.preventDefault();
      if (!activeKeysRef.current.has(key)) {
        activeKeysRef.current.add(key);
        setActiveKeys(new Set(activeKeysRef.current));
        updateKeyboardVelocity();
      }
    }
  };

  const handleKeyUp = (e: KeyboardEvent) => {
    if (!isManualMode) return;
    const key = e.key.toLowerCase();
    if (['w', 'a', 's', 'd'].includes(key)) {
      e.preventDefault();
      if (activeKeysRef.current.has(key)) {
        activeKeysRef.current.delete(key);
        setActiveKeys(new Set(activeKeysRef.current));
        updateKeyboardVelocity();
      }
    }
  };

  useEffect(() => {
    if (isDragging || isTurretDragging) {
      document.addEventListener('mousemove', handleMouseMove);
      document.addEventListener('mouseup', handleMouseUp);
    }

    return () => {
      document.removeEventListener('mousemove', handleMouseMove);
      document.removeEventListener('mouseup', handleMouseUp);
    };
  }, [isDragging, isTurretDragging]);

  useEffect(() => {
    if (isManualMode) {
      document.addEventListener('keydown', handleKeyDown);
      document.addEventListener('keyup', handleKeyUp);
    }
    return () => {
      document.removeEventListener('keydown', handleKeyDown);
      document.removeEventListener('keyup', handleKeyUp);
    };
  }, [isManualMode]);

  useEffect(() => {
    if (!isManualMode) {
      activeKeysRef.current.clear();
      setActiveKeys(new Set());
      setKeyboardVelocity({ x: 0, y: 0 });
      publishVelocity(0, 0);
    }
  }, [isManualMode]);

  return (
    <Card className="military-panel">
      <CardHeader className="pb-2">
        <CardTitle className="text-military-red text-sm">CONTROL INTERFACE</CardTitle>
      </CardHeader>

      <CardContent className="space-y-6">

        {/* Mode Switch */}
        <div className="flex items-center justify-between">
          <span className="text-xs text-gray-400">AUTONOMOUS</span>
          <Switch
            checked={isManualMode}
            onCheckedChange={setIsManualMode}
            className="data-[state=checked]:bg-military-red"
          />
          <span className="text-xs text-gray-400">MANUAL</span>
        </div>

        {/* Bot Movement */}
        {isManualMode && (
          <div className="flex flex-col space-y-4">
            <div className="flex justify-center gap-32">
              {/* Bot Movement */}
              <div className="space-y-2">
                <p className="text-xs text-military-red font-semibold text-center">BOT MOVEMENT</p>
                <div className="flex justify-center">
                  <div
                    ref={joystickRef}
                    className="joystick-area cursor-pointer relative w-32 h-32 bg-black bg-opacity-20 rounded-full"
                    onMouseDown={handleMouseDown}
                  >
                    <div
                      className="joystick-knob absolute w-8 h-8 bg-military-red rounded-full"
                      style={{
                        transform: `translate(calc(-50% + ${joystickPosition.x}px), calc(-50% + ${joystickPosition.y}px))`,
                        left: '50%',
                        top: '50%',
                        cursor: isDragging ? 'grabbing' : 'grab',
                      }}
                    />
                    <div className="absolute inset-0 flex items-center justify-center pointer-events-none">
                      <div className="w-full h-px bg-military-red opacity-30"></div>
                    </div>
                    <div className="absolute inset-0 flex items-center justify-center pointer-events-none">
                      <div className="h-full w-px bg-military-red opacity-30"></div>
                    </div>
                  </div>
                </div>
                <div className="text-xs text-center text-gray-400">WASD or drag to move</div>
              </div>

              {/* Turret Control */}
              <div className="space-y-2">
                <p className="text-xs text-military-red font-semibold text-center">TURRET CONTROL</p>
                <div className="flex justify-center">
                  <div
                    ref={turretJoystickRef}
                    className="joystick-area cursor-pointer relative w-32 h-32 bg-black bg-opacity-20 rounded-full"
                    onMouseDown={(e) => {
                      setIsTurretDragging(true);
                      handleTurretMove(e.clientX, e.clientY);
                    }}
                  >
                    <div
                      className="joystick-knob absolute w-8 h-8 bg-military-amber rounded-full"
                      style={{
                        transform: `translate(calc(-50% + ${turretPosition.x}px), calc(-50% + ${turretPosition.y}px))`,
                        left: '50%',
                        top: '50%',
                        cursor: isTurretDragging ? 'grabbing' : 'grab',
                      }}
                    />
                    <div className="absolute inset-0 flex items-center justify-center pointer-events-none">
                      <div className="w-full h-px bg-military-amber opacity-30"></div>
                    </div>
                    <div className="absolute inset-0 flex items-center justify-center pointer-events-none">
                      <div className="h-full w-px bg-military-amber opacity-30"></div>
                    </div>
                  </div>
                </div>
                <div className="text-xs text-center text-gray-400">Aim turret</div>
                <div className="flex justify-center">
                  <Button
                    onClick={() => sendQuickAction('/fire')}
                    size="sm"
                    variant="destructive"
                    className="text-xs px-4"
                  >
                    FIRE
                  </Button>
                </div>
              </div>
            </div>
          </div>
        )}

        {/* Quick Actions */}
        <div className="space-y-2">
          <p className="text-xs text-military-red font-semibold">QUICK ACTIONS</p>
          <div className="grid grid-cols-2 gap-2">
            <Button onClick={() => sendQuickAction('/reset_odometry')} variant="outline" size="sm" className="text-xs border-military-border">
              RESET ODOMETRY
            </Button>
            <Button onClick={() => sendQuickAction('/get_map')} variant="outline" size="sm" className="text-xs border-military-border">
              GET MAP
            </Button>
            <Button onClick={() => sendQuickAction('/follow_target')} variant="outline" size="sm" className="text-xs border-military-border">
              FOLLOW TARGET
            </Button>
            <Button
              disabled={!targetSelected}
              onClick={() => sendQuickAction('/confirm_target')}
              variant="outline"
              size="sm"
              className="text-xs border-military-border"
            >
              CONFIRM TARGET
            </Button>
          </div>
        </div>

      </CardContent>
    </Card>
  );
};

export default ControlPanel;
