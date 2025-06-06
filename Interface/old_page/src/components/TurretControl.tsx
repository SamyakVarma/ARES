import React, { useState } from 'react';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Badge } from '@/components/ui/badge';
import { Button } from '@/components/ui/button';
import { Joystick } from 'react-joystick-component';

const TurretControl: React.FC = () => {
  const [status, setStatus] = useState<'idle' | 'aiming' | 'firing'>('idle');

  const handleAimMove = (event: any) => {
    setStatus('aiming');
    fetch('http://localhost:5000/turret_cmd', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        pan: event.x || 0,
        tilt: event.y || 0,
      }),
    });
  };

  const handleAimStop = () => {
    setStatus('idle');
    fetch('http://localhost:5000/turret_cmd', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ pan: 0, tilt: 0 }),
    });
  };

  const handleFire = () => {
    setStatus('firing');
    fetch('http://localhost:5000/fire', {
      method: 'POST',
    }).finally(() => {
      setTimeout(() => setStatus('idle'), 1000);
    });
  };

  return (
    <Card className="military-panel w-full max-w-md mx-auto">
      <CardHeader className="pb-2">
        <div className="flex items-center justify-between">
          <CardTitle className="text-military-red text-sm">TURRET CONTROL</CardTitle>
          <Badge variant="outline" className="text-xs">
            {status.toUpperCase()}
          </Badge>
        </div>
      </CardHeader>

      <CardContent className="flex flex-col items-center space-y-4">
        <Joystick
          size={80}
          baseColor="#2c2c2c"
          stickColor="#ff4545"
          move={handleAimMove}
          stop={handleAimStop}
          throttle={100}
        />
        <Button variant="destructive" onClick={handleFire} className="w-full">
          FIRE
        </Button>
      </CardContent>
    </Card>
  );
};

export default TurretControl;
