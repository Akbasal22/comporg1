call C:\Xilinx_comporg\Vivado\2017.4\settings64.bat

set folder=Simulation_Files
mkdir %folder%
copy "RAM.mem" "%folder%/RAM.mem"
cd "%folder%

@REM ::Register 16 Simulation
@REM call xvlog ../Register16bit.v  
@REM call xvlog ../Register16bitSimulation.v
@REM call xvlog ../Helper.v
@REM call xelab -top Register16bitSimulation -snapshot reg16sim -debug typical
@REM call xsim reg16sim -R

@REM ::Register 32 Simulation
@REM call xvlog ../Register32bit.v  
@REM call xvlog ../Register32bitSimulation.v
@REM call xvlog ../Helper.v
@REM call xelab -top Register32bitSimulation -snapshot reg32sim -debug typical
@REM call xsim reg32sim -R

@REM ::Register File Simulation
@REM call xvlog ../Register32bit.v  
@REM call xvlog ../RegisterFile.v  
@REM call xvlog ../RegisterFileSimulation.v
@REM call xvlog ../Helper.v
@REM call xelab -top RegisterFileSimulation -snapshot regfilesim -debug typical
@REM call xsim regfilesim -R

@REM ::Address Register File Simulation
@REM call xvlog ../Register16bit.v  
@REM call xvlog ../AddressRegisterFile.v  
@REM call xvlog ../AddressRegisterFileSimulation.v
@REM call xvlog ../Helper.v
@REM call xelab -top AddressRegisterFileSimulation -snapshot addregfilesim -debug typical
@REM call xsim addregfilesim -R

@REM ::Instruction Register Simulation
@REM  call xvlog ../InstructionRegister.v  
@REM  call xvlog ../InstructionRegisterSimulation.v
@REM  call xvlog ../Helper.v
@REM  call xelab -top InstructionRegisterSimulation -snapshot insregsim -debug typical
@REM  call xsim insregsim -R

@REM  ::Data Register Simulation
@REM  call xvlog ../DataRegister.v  
@REM  call xvlog ../DataRegisterSimulation.v
@REM  call xvlog ../Helper.v
@REM  call xelab -top DataRegisterSimulation -snapshot dataregsim -debug typical
@REM  call xsim dataregsim -R

@REM ::Arithmetic Logic Unit Simulation
@REM  call xvlog ../ArithmeticLogicUnit.v  
@REM  call xvlog ../ArithmeticLogicUnitSimulation.v
@REM  call xvlog ../Helper.v
@REM  call xelab -top ArithmeticLogicUnitSimulation -snapshot alusim -debug typical
@REM  call xsim alusim -R

::Arithmetic Logic Unit System Simulation
 call xvlog ../Register16bit.v  
 call xvlog ../Register32bit.v  
 call xvlog ../RegisterFile.v
 call xvlog ../AddressRegisterFile.v  
 call xvlog ../InstructionRegister.v
 call xvlog ../DataRegister.v  
 call xvlog ../ArithmeticLogicUnit.v
 call xvlog ../Memory.v  
 call xvlog ../ArithmeticLogicUnitSystem.v  
 call xvlog ../ArithmeticLogicUnitSystemSimulation.v
 call xvlog ../Helper.v

 call xelab -top ArithmeticLogicUnitSystemSimulation -snapshot alusyssim -debug typical
 call xsim alusyssim -R

cd ..
