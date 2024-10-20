import React, { useState, useEffect, useRef } from 'react';

// Component Name: ExampleComponent
function DriveStatePanel() {
  const [driveState, setDriveState] = useState('Idle');


    const CommandButton = ({ label, active, onClick }) => (
        <button className={"command-button " + active} onClick={onClick}>
          {label}
        </button>
      );
 

  // Render the component UI
  return (
    <div className="panel">
            <h2 className="panel-title">Controls</h2>
            <div className="command-grid">
              {['Autonomous Drive', 'Direct Drive', 'Idle'].map((label, index) => (
                <CommandButton key={index} label={label} 
                active={label === driveState ? 'active' : ''}
                onClick={() => setDriveState(label)}
                />
              ))}
            </div>
            <button className="estop-button">
              ESTOP
            </button>
          </div>
  );
}

export default DriveStatePanel;
