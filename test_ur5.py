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
	#insert code here??
    except:
        traceback.print_exc()
    finally:
        rob.close()
