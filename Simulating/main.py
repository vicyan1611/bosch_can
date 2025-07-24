from Simulator import Simulator

def main():
    simulator = Simulator()
    simulator.run_simulation()
    # simulator.run_simulation_local()
    simulator.plot_results()

if __name__ == "__main__":
    main()