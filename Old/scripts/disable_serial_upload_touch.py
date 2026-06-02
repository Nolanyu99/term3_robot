Import("env")

board = env.BoardConfig()
board.update("upload.disable_flushing", True)
board.update("upload.use_1200bps_touch", False)
board.update("upload.wait_for_upload_port", False)
