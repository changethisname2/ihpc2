import urx
from IPython import embed
import logging
import traceback

if __name__ == "__main__":
    try:
        print("Connecting to Robot...")
        logging.basicConfig(level=logging.WARN)
        while True:
            try:
                print("...")
                rob = urx.Robot("192.168.1.6")
                rob.set_tcp((0, 0, 0.335, 0, 0, 0))
                rob.set_payload(0.5, (0, 0, 0))
                break
            except KeyboardInterrupt:
                break
            except:
                try:
	                rob.close()
                except:
	                pass
        embed()
    except:
        traceback.print_exc()
    finally:
        rob.close()
      
       #point4 = point3[:]

       #point4[2] += -0.08

       #rob.movel(point4, 0.4, 0.3)

'''
#a good position right above the board       
Transform:
<Orientation: 
array([[-0.09616118, -0.99501282,  0.02650485],
       [-0.99265482,  0.09390132, -0.07628206],
       [ 0.07341278, -0.03364554, -0.99673394]])>
<Vector: (0.58576, 0.35213, -0.07606)>

'''
