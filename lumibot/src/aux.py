class Aux:

    @staticmethod
    def print(string, color='white'):
        colors = {
            "red": "\033[91m",
            "green": "\033[92m",
            "yellow": "\033[93m",
            "blue": "\033[94m",
            "magenta": "\033[95m",
            "cyan": "\033[96m",
            "white": "\033[97m",
            "black": "\033[98m",
        }
        print(f"{colors[color]}{string}\033[00m")