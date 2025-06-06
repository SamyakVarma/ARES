
import React, { useState, useRef, useEffect } from 'react';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { Switch } from '@/components/ui/switch';
import { Badge } from '@/components/ui/badge';
import ControlPanel from '@/components/ControlPanel';
import CameraFeed from '@/components/CameraFeed';
import MountedCameraFeed from '@/components/MountedCameraFeed';
import MapVisualization from '@/components/MapVisualization';
import FaceDetection from '@/components/FaceDetection';
import SystemStatus from '@/components/SystemStatus';

const Index = () => {
  const [isManualMode, setIsManualMode] = useState(false);
  const [selectedTarget, setSelectedTarget] = useState<string | null>(null);
  const [connectionStatus, setConnectionStatus] = useState('connected');

  return (
    <div className="min-h-screen bg-military-bg">
      <div className="relative border-b-2 border-military-red header-glow">
        <div className="flex items-center justify-center py-6">
          <div className="flex items-center space-x-4">
            <div className="w-16 h-16 bg-military-red rounded-lg flex items-center justify-center military-glow">
              <span className="text-3xl font-bold text-white"><img src='ARES.png'></img></span>
            </div>
            <div className="text-center">
              <h1 className="text-4xl font-bold text-military-red font-mono text-glow">A.R.E.S.</h1>
              <p className="text-sm text-gray-400">Autonomous Recon and Elimination System</p>
            </div>
          </div>
        </div>
      </div>

      <div className="p-4">
        <div className="grid grid-cols-12 gap-4 h-[calc(100vh-10rem)]">
          {/* Navigation Camera - Far Left */}
          <div className="col-span-4 flex items-start justify-center">
            <div className="w-full max-w-[500px] aspect-video card-glow">
              <CameraFeed />
            </div>
          </div>

          {/* Control Interface - Center */}
          <div className="col-span-4 space-y-4">
            <div className="card-glow">
              <ControlPanel 
                isManualMode={isManualMode}
                setIsManualMode={setIsManualMode}
              />
            </div>
            
            {/* Mission Status */}
            <Card className="military-panel card-glow">
              <CardHeader className="pb-2">
                <CardTitle className="text-military-red text-sm text-glow">MISSION STATUS</CardTitle>
              </CardHeader>
              <CardContent className="space-y-2">
                <div className="flex justify-between text-xs">
                  <span className="text-gray-400">Target:</span>
                  <span className="text-military-red text-glow">{selectedTarget || 'None Selected'}</span>
                </div>
                <div className="flex justify-between text-xs">
                  <span className="text-gray-400">Mode:</span>
                  <Badge variant={isManualMode ? "destructive" : "default"} className="text-xs military-glow">
                    {isManualMode ? 'MANUAL' : 'AUTONOMOUS'}
                  </Badge>
                </div>
                <div className="flex justify-between text-xs">
                  <span className="text-gray-400">Status:</span>
                  <span className="text-military-green text-glow">OPERATIONAL</span>
                </div>
              </CardContent>
            </Card>

            {/* Map */}
            <div className="flex-1 card-glow">
              <MapVisualization />
            </div>
          </div>

          {/* Mounted Camera - Far Right */}
          <div className="col-span-4 flex items-start justify-center">
            <div className="w-full max-w-[500px] aspect-video card-glow">
              <MountedCameraFeed />
            </div>
          </div>
        </div>
      </div>


    </div>
  );
};

export default Index;
