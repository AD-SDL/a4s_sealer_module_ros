"""The server that takes incoming WEI flow requests from the experiment application"""
"""The server that takes incoming WEI flow requests from the experiment application"""
import json
from argparse import ArgumentParser
from contextlib import asynccontextmanager

from fastapi import FastAPI, File, Form, UploadFile
from fastapi.responses import JSONResponse
from a4s_sealer_driver.a4s_sealer_driver import A4S_SEALER_DRIVER  # import sealer driver
import time

# TODO: db backup of tasks and results (can be a proper db or just a file)
# TODO logging for server and workcell
# TODO consider sub-applications for different parts of the server (e.g. /job, /queue, /data, etc.)
# TODO make the workcell live in the DATA_DIR and be coupled to the server
#      This might entail making a rq object of the wei object and making that available to the workers

workcell = None
global sealer
Port = '/dev/ttyUSB1'

@asynccontextmanager
async def lifespan(app: FastAPI):
    global sealer
    """Initial run function for the app, parses the worcell argument
        Parameters
        ----------
        app : FastApi
           The REST API app being initialized

        Returns
        -------
        None"""
    try:
            sealer = A4S_SEALER_DRIVER(Port)
    except Exception as err:
            state = "SEALER CONNECTION ERROR"

    # Yield control to the application
    yield

    # Do any cleanup here
    pass


app = FastAPI(lifespan=lifespan, )


@app.post("/action")
async def do_action(
    action_handle: str,
    action_vars: dict, 
):
    global sealer
    if action_handle == 'seal':  
        #self.sealer.set_time(3)
        #self.sealer.set_temp(175)
        try: 
            time = vars.get('time',3)
            temp = vars.get('temp',175)

            sealer.seal()
            time.sleep(15)  
            response_content = {
                    "action_msg": "StepStatus.Succeeded",
                    "action_response": "True",
                    "action_log": ""
                    
                }
            return JSONResponse(content=response_content)
        except Exception as e:
            response_content = {
            "status": "failed",
            "error": str(e),
        }
            return JSONResponse(content=response_content)
@app.post("/action")
async def do_action(
    action_handle: str,
    action_vars: dict, 
):
    global sealer
    if action_handle == 'seal':  
        #self.sealer.set_time(3)
        #self.sealer.set_temp(175)
        try: 
            time = vars.get('time',3)
            temp = vars.get('temp',175)

            sealer.seal()
            time.sleep(15)  
            response_content = {
                    "action_msg": "StepStatus.Succeeded",
                    "action_response": "True",
                    "action_log": ""
                    
                }
            return JSONResponse(content=response_content)
        except Exception as e:
            response_content = {
            "status": "failed",
            "error": str(e),
        }
            return JSONResponse(content=response_content)

@app.post("/state")
async def state(
    action_handle: str,
    action_vars: dict, 
):
    global sealer
    return JSONResponse(content={"State": sealer.get_status() })

if __name__ == "__main__":
    import uvicorn
    print("asdfsaf")
    uvicorn.run("rpl_wei.server:app", reload=True, ws_max_size=100000000000000000000000000000000000000)
