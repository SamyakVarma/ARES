import React, { useState } from 'react';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Badge } from '@/components/ui/badge';
import ControlPanel from '@/components/ControlPanel';
import CameraFeed from '@/components/CameraFeed';
import MountedCameraFeed from '@/components/MountedCameraFeed';
import MapVisualization from '@/components/MapVisualization';

const Index = () => {
  const [isManualMode, setIsManualMode] = useState(false);
  const [selectedTarget, setSelectedTarget] = useState<string | null>(null);

  return (
    <div className="min-h-screen bg-military-bg">
      {/* Header */}
      <div className="relative border-b-2 border-military-red header-glow">
        <div className="flex items-center justify-center py-6">
          <div className="flex items-center space-x-4">
            <div className="w-16 h-16 bg-military-red rounded-lg flex items-center justify-center military-glow">
              <span className="text-3xl font-bold text-white">
                <img src='ARES.png' alt="ARES" />
              </span>
            </div>
            <div className="text-center">
              <h1 className="text-4xl font-bold text-military-red font-mono text-glow">A.R.E.S.</h1>
              <p className="text-sm text-gray-400">Autonomous Recon and Elimination System</p>
            </div>
          </div>
        </div>
      </div>

      {/* Main Grid */}
      <div className="p-4">
        <div className="grid grid-cols-12 gap-4 h-[calc(100vh-10rem)]">
          
          {/* Left Column: Navigation Camera + Map */}
          <div className="col-span-4 flex-1 flex-col space-y-4 items-center">
            <div className="w-full max-w-[500px] aspect-video card-glow">
              <CameraFeed />
            </div>
            <div className="w-full max-w-[500px] flex-1 card-glow overflow-hidden">
              <MapVisualization />
            </div>
          </div>

          {/* Center Column: Controls + Status */}
          <div className="col-span-4 space-y-4">
            {/* Control Panel */}
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
          </div>

          {/* Right Column: Mounted Camera */}
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
