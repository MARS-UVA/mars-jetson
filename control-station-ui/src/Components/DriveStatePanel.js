/**
 * DriveStatePanel Component
 * 
 * This component renders a control panel with buttons to switch between different drive states
 * and an emergency stop (ESTOP) button.
 */
import React, {useState, useEffect, useRef} from 'react';
function DriveStatePanel() {
  // Define the drive state and displays the current state
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
