"""The server that takes incoming WEI flow requests from the experiment application"""
import json
from argparse import ArgumentParser
from contextlib import asynccontextmanager
import time
from fastapi import FastAPI, File, Form, UploadFile
from fastapi.responses import JSONResponse

#from a4s_sealer_driver.a4s_sealer_driver import A4S_SEALER_DRIVER  # import sealer driver

workcell = None
global sealer, state
serial_port = '/dev/ttyUSB0'
local_ip = 'parker.alcf.anl.gov'
local_port = '8000'


@asynccontextmanager
async def lifespan(app: FastAPI):
    global sealer, state
    """Initial run function for the app, parses the worcell argument
        Parameters
        ----------
        app : FastApi
           The REST API app being initialized

        Returns
        -------
        None"""
    try:
            sealer = A4S_SEALER_DRIVER(serial_port)
            state = "IDLE"
    except Exception as err:
            state = " ERROR"

    # Yield control to the application
    yield

    # Do any cleanup here
    pass


app = FastAPI(lifespan=lifespan, )

@app.get("/state")
def get_state():
    global sealer, state
    return JSONResponse(content={"State": state})#sealer.get_status() })

@app.get("/description")
async def description():
    global sealer, state
    return JSONResponse(content={"State": state })#sealer.get_status() })

@app.get("/resources")
async def resources():
    global sealer, state
    return JSONResponse(content={"State": state })#sealer.get_status() })


@app.post("/action")
def do_action(
    action_handle: str,
    action_vars: str, 
):

    global sealer, state
    state = "BUSY"
    if action_handle == 'seal':  
        #self.sealer.set_time(3)
        #self.sealer.set_temp(175)
        try: 
          
            sealer.seal()
            time.sleep(15)  
            response_content = {
                    "action_msg": "StepStatus.Succeeded",
                    "action_response": "True",
                    "action_log": ""
                    
                    
                }
            state = "IDLE"
            return JSONResponse(content=response_content)
        except Exception as e:
            response_content = {
            "status": "failed",
            "error": str(e),
        }
            state = "IDLE"
            return JSONResponse(content=response_content)
   

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("a4s_sealer_REST:app", host=local_ip, port=local_port, reload=True, ws_max_size=100000000000000000000000000000000000000)
