**What do I do?**
This is a program written by Mehmet Kara aimed at solving the forces applied to the following components:
    - Both forward control arms (forces at outboard node + damper force for lower)
    - Rear trailing arm (force at outboard node + tie rod + damper force)

This program is design to run on a vehicle with the following suspension setup:
 - Double wishbone front suspension with direct acting coil over dampers
 - Trailing arm rear suspension with tie rod node + "KTM style"  rocker

 The program allows for easy adjustment of these nodes, as long as the suspension concept stays the same.
If the suspension concept doesnt match the one above, the program must be changed/

The program obtains its braking and cornering load casing from a output from CarSim 
    - Created using the solar car CarSim model
    - The vehicle does a 45 degree step steer @ 50kph before slamming the brakes afterwards
    - The CarSim data has forces and moments at the wheel center

The provided CarSim data is still valid as long as:
    - The mass of the vehicle, as well as its wheelbase and track dont change at all
    - The suspension geometry doesnt change a longitudinal
If these values do change, a new run must be done with the data output in the same format as before.

After running, the program will output the cartesian components for component loads + 3D plots

If support is needed for this program, text me on the Buckeye Solar Racing discord : )

**How to run me?**

1. Create a virtual enviroment

    put the following lines into the terminal if running Linux/macOS...

        #python3 -m venv venv

        #source venv/bin/activate 

    and if youre running windows, open powershell and run...

        #python3 -m venv venv

        #venv\Scripts\Activate.ps1

2. Install libraries

    all libraries are listed in the requirements.txt so all you must do to install them is put the following line into the terminal...
    
    # pip install -r requirements.txt


