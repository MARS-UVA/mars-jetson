import React, { useState, useEffect, useRef } from 'react';

// Component Name: ExampleComponent
function GamepadPanel() {
  // State Hook - useState for managing local state
  const [count, setCount] = useState(0);
  const [gamepadStatus, setGamepadStatus] = useState('No gamepad connected!');
  useEffect(() => {
    const handleGamepadConnected = (e) => {
      setGamepadStatus(`Gamepad connected!: ${e.gamepad.id}`);
    };

    const handleGamepadDisconnected = () => {
      setGamepadStatus('No gamepad connected!');
    };

    window.addEventListener('gamepadconnected', handleGamepadConnected);
    window.addEventListener('gamepaddisconnected', handleGamepadDisconnected);

    return () => {
      window.removeEventListener('gamepadconnected', handleGamepadConnected);
      window.removeEventListener('gamepaddisconnected', handleGamepadDisconnected);
    };
  }, []);
  


  // Render the component UI
  return (
    <div className="panel">
            {/* <h2 className="panel-title">Gamepad Status</h2> */}
            <p className="gamepad-status">{gamepadStatus}</p>
    </div>
  );
}

export default GamepadPanel;
