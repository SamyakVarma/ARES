
import React, { useState, useEffect } from 'react';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { Badge } from '@/components/ui/badge';

interface FaceDetectionProps {
  selectedTarget: string | null;
  setSelectedTarget: (target: string | null) => void;
}

interface DetectedFace {
  id: string;
  confidence: number;
  timestamp: string;
  status: 'identified' | 'unknown' | 'target';
  lastSeen: string;
}

const FaceDetection: React.FC<FaceDetectionProps> = ({ selectedTarget, setSelectedTarget }) => {
  const [detectedFaces, setDetectedFaces] = useState<DetectedFace[]>([
    {
      id: 'FACE_001',
      confidence: 0.94,
      timestamp: '14:32:15',
      status: 'identified',
      lastSeen: '2s ago'
    },
    {
      id: 'FACE_002',
      confidence: 0.87,
      timestamp: '14:31:42',
      status: 'unknown',
      lastSeen: '45s ago'
    },
    {
      id: 'FACE_003',
      confidence: 0.91,
      timestamp: '14:30:18',
      status: 'identified',
      lastSeen: '2m ago'
    }
  ]);

  const handleSelectTarget = (faceId: string) => {
    setSelectedTarget(faceId === selectedTarget ? null : faceId);
    
    setDetectedFaces(prev => prev.map(face => ({
      ...face,
      status: face.id === faceId ? 'target' : face.status
    })));
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'target': return 'bg-military-red';
      case 'identified': return 'bg-military-green';
      default: return 'bg-military-amber';
    }
  };

  const getStatusText = (status: string) => {
    switch (status) {
      case 'target': return 'TARGET';
      case 'identified': return 'KNOWN';
      default: return 'UNKNOWN';
    }
  };

  return (
    <Card className="military-panel h-full">
      <CardHeader className="pb-2">
        <div className="flex items-center justify-between">
          <CardTitle className="text-military-red text-sm">TARGET ACQUISITION</CardTitle>
          <div className="flex items-center space-x-2">
            <div className="status-indicator bg-military-green" />
            <Badge variant="outline" className="text-xs">
              {detectedFaces.length} DETECTED
            </Badge>
          </div>
        </div>
      </CardHeader>
      <CardContent className="h-full pb-2 overflow-y-auto">
        <div className="space-y-3">
          <div className="p-2 bg-military-bg rounded border border-military-border">
            <div className="text-xs text-military-red font-semibold mb-1">CURRENT TARGET</div>
            <div className="text-xs text-gray-400">
              {selectedTarget ? selectedTarget : 'No target selected'}
            </div>
          </div>

          <div className="space-y-2">
            <div className="text-xs text-military-red font-semibold">DETECTED FACES</div>
            
            {detectedFaces.map((face) => (
              <div
                key={face.id}
                className={`p-3 rounded border cursor-pointer transition-all ${
                  selectedTarget === face.id 
                    ? 'border-military-red bg-military-red bg-opacity-10' 
                    : 'border-military-border hover:border-military-red hover:border-opacity-50'
                }`}
                onClick={() => handleSelectTarget(face.id)}
              >
                <div className="flex items-start space-x-3">
                  <div className="w-12 h-12 bg-gray-700 rounded border border-military-border flex items-center justify-center">
                    <span className="text-xs text-gray-400">IMG</span>
                  </div>
                  
                  <div className="flex-1 space-y-1">
                    <div className="flex items-center justify-between">
                      <span className="text-xs font-mono text-white">{face.id}</span>
                      <Badge 
                        variant="outline" 
                        className={`text-xs ${getStatusColor(face.status)} text-white border-0`}
                      >
                        {getStatusText(face.status)}
                      </Badge>
                    </div>
                    
                    <div className="text-xs text-gray-400 space-y-1">
                      <div className="flex justify-between">
                        <span>Confidence:</span>
                        <span className="text-military-green">{(face.confidence * 100).toFixed(1)}%</span>
                      </div>
                      <div className="flex justify-between">
                        <span>First seen:</span>
                        <span>{face.timestamp}</span>
                      </div>
                      <div className="flex justify-between">
                        <span>Last seen:</span>
                        <span>{face.lastSeen}</span>
                      </div>
                    </div>
                    
                    {selectedTarget === face.id && (
                      <Button
                        size="sm"
                        className="w-full mt-2 bg-military-red hover:bg-red-700 text-xs"
                        onClick={(e) => {
                          e.stopPropagation();
                          console.log(`TARGET LOCKED: ${face.id}`);
                        }}
                      >
                        ENGAGE TARGET
                      </Button>
                    )}
                  </div>
                </div>
              </div>
            ))}
          </div>
        </div>
      </CardContent>
    </Card>
  );
};

export default FaceDetection;
