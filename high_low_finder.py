import json

# Input data (replace this with reading from a file if needed)
data = {
    "LAPTOP-LAPTOP_GODOTROS": {
        "simple_string": {
            "Packet Loss Percentage": 0.0,
            "Average RTT": 0.017432430065332738,
            "Median RTT": 0.014069975000287102,
            "Standard Deviation of RTT": 0.006249725144911916,
            "Variance of RTT": 3.916485256717739e-05,
            "% Under Threshold": 67.9,
            "% Over Threshold": 32.1
        },
        "increasing_payload": {
            "Packet Loss Percentage": 89.11,
            "Average RTT": 0.019604415692630967,
            "Median RTT": 0.018320714166293283,
            "Standard Deviation of RTT": 0.004210821241489737,
            "Variance of RTT": 1.842040368417816e-05,
            "% Under Threshold": 8.503333333333334,
            "% Over Threshold": 2.3833333333333333
        },
        "large_payload": {
            "Packet Loss Percentage": 100.0
        }
    },
    "HEADSET-TB3_GODOTROS": {
        "simple_string": {
            "Packet Loss Percentage": 0.0,
            "Average RTT": 0.0937954461983287,
            "Median RTT": 0.07329470900003794,
            "Standard Deviation of RTT": 0.06905914293948212,
            "Variance of RTT": 0.004770590132615173,
            "% Under Threshold": 2.9333333333333336,
            "% Over Threshold": 97.06666666666666
        },
        "large_payload": {
            "Packet Loss Percentage": 100.0
        },
        "increasing_payload": {
            "Packet Loss Percentage": 89.11,
            "Average RTT": 0.07117256166146524,
            "Median RTT": 0.06515623149986514,
            "Standard Deviation of RTT": 0.03018101957836279,
            "Variance of RTT": 0.0009118372831133902,
            "% Under Threshold": 0.016666666666666666,
            "% Over Threshold": 10.87
        }
    },
    "LAPTOP-LAPTOP_ROS2UNITY": {
        "large_payload": {
            "Packet Loss Percentage": 0.0,
            "Average RTT": 0.01668732421600266,
            "Median RTT": 0.014903679500093817,
            "Standard Deviation of RTT": 0.0068664118895696395,
            "Variance of RTT": 4.717258304146985e-05,
            "% Under Threshold": 63.06666666666667,
            "% Over Threshold": 36.93333333333333
        },
        "simple_string": {
            "Packet Loss Percentage": 0.0,
            "Average RTT": 0.0017315267350031215,
            "Median RTT": 0.0016723006668446668,
            "Standard Deviation of RTT": 0.001303198032631895,
            "Variance of RTT": 2.0605277812451554e-06,
            "% Under Threshold": 99.89999999999999,
            "% Over Threshold": 0.10000000000000002
        },
        "increasing_payload": {
            "Packet Loss Percentage": 0.0,
            "Average RTT": 0.01055544071032874,
            "Median RTT": 0.0078157829999933,
            "Standard Deviation of RTT": 0.007799145735189358,
            "Variance of RTT": 6.0854517281992404e-05,
            "% Under Threshold": 87.39999999999999,
            "% Over Threshold": 12.6
        }
    },
    "HEADSET-LAPTOP_GODOTROS": {
        "simple_string": {
            "Packet Loss Percentage": 0.0,
            "Average RTT": 0.09165208752033198,
            "Median RTT": 0.07088738866665756,
            "Standard Deviation of RTT": 0.06973359227871621,
            "Variance of RTT": 0.0048635000126008455,
            "% Under Threshold": 4.8,
            "% Over Threshold": 95.2
        },
        "increasing_payload": {
            "Packet Loss Percentage": 89.11,
            "Average RTT": 0.05624601443229216,
            "Median RTT": 0.046899110833389046,
            "Standard Deviation of RTT": 0.03128890620284349,
            "Variance of RTT": 0.001024639583618467,
            "% Under Threshold": 0.0,
            "% Over Threshold": 10.89
        }
    },
    "HEADSET-TB3_UNITYTCPCONN": {
        "simple_string": {
            "Packet Loss Percentage": 0.0,
            "Average RTT": 0.07726364963066502,
            "Median RTT": 0.05859790349995814,
            "Standard Deviation of RTT": 0.057026877426395305,
            "Variance of RTT": 0.003597956525799279,
            "% Under Threshold": 2.1999999999999997,
            "% Over Threshold": 97.8
        },
        "increasing_payload": {
            "Packet Loss Percentage": 39.1025,
            "Average RTT": 13.643077953866715,
            "Median RTT": 2.098955281749909,
            "Standard Deviation of RTT": 18.99413495221914,
            "Variance of RTT": 401.39602826772125,
            "% Under Threshold": 0.0,
            "% Over Threshold": 60.89750000000001
        },
        "large_payload": {
            "Packet Loss Percentage": 65.8475,
            "Average RTT": 42.34168442573678,
            "Median RTT": 39.932855643999915,
            "Standard Deviation of RTT": 21.5808618760957,
            "Variance of RTT": 624.686577400348,
            "% Under Threshold": 0.0,
            "% Over Threshold": 34.152499999999996
        }
    },
    "HEADSET-LAPTOP_UNITYTCPCONN": {
        "simple_string": {
            "Packet Loss Percentage": 18.324,
            "Average RTT": 0.1400023922706966,
            "Median RTT": 0.08506881779994722,
            "Standard Deviation of RTT": 0.31919869163131237,
            "Variance of RTT": 0.3408279102256738,
            "% Under Threshold": 1.556,
            "% Over Threshold": 80.122
        },
        "increasing_payload": {
            "Packet Loss Percentage": 28.599999999999998,
            "Average RTT": 0.32311882092435423,
            "Median RTT": 0.13899329799978655,
            "Standard Deviation of RTT": 0.4489787395427586,
            "Variance of RTT": 0.3886265929774567,
            "% Under Threshold": 0.08,
            "% Over Threshold": 99.88000000000001
        },
        "large_payload": {
            "Packet Loss Percentage": 50.56666666666666,
            "Average RTT": 20.943964746027365,
            "Median RTT": 21.117109342499816,
            "Standard Deviation of RTT": 12.09699511180606,
            "Variance of RTT": 147.81859474621658,
            "% Under Threshold": 0.0,
            "% Over Threshold": 74.15
        }
    },
    "LAPTOP-LAPTOP_UNITYTCPCONN": {
        "increasing_payload": {
            "Packet Loss Percentage": 0.0,
            "Average RTT": 0.016784783161008163,
            "Median RTT": 0.013080745500095218,
            "Standard Deviation of RTT": 0.010113947061134141,
            "Variance of RTT": 0.00010253355689291976,
            "% Under Threshold": 66.63333333333333,
            "% Over Threshold": 33.36666666666667
        },
        "large_payload": {
            "Packet Loss Percentage": 0.0,
            "Average RTT": 0.025664381675326947,
            "Median RTT": 0.025451140833562567,
            "Standard Deviation of RTT": 0.008361849056569346,
            "Variance of RTT": 6.992724605071276e-05,
            "% Under Threshold": 29.03333333333333,
            "% Over Threshold": 70.96666666666665
        },
        "simple_string": {
            "Packet Loss Percentage": 0.0,
            "Average RTT": 0.0076850551156724374,
            "Median RTT": 0.007622602500001983,
            "Standard Deviation of RTT": 0.003030781617536211,
            "Variance of RTT": 9.22320163658602e-06,
            "% Under Threshold": 99.86666666666667,
            "% Over Threshold": 0.13333333333333333
        }
    }
}

def find_extreme_rtts(data):
    # Dictionary to store results
    results = {}
    
    for environment, tests in data.items():
        for test_type, metrics in tests.items():
            # Skip if no "Average RTT" is present (e.g., for 100% packet loss)
            if "Average RTT" not in metrics:
                continue
            
            # Convert Average RTT from seconds to milliseconds
            avg_rtt_ms = metrics["Average RTT"] * 1000
            
            # Initialize the dictionary if test_type is encountered for the first time
            if test_type not in results:
                results[test_type] = {
                    "highest": {"environment": None, "value": float('-inf')},
                    "lowest": {"environment": None, "value": float('inf')}
                }
            
            # Update the highest RTT if current value is greater
            if avg_rtt_ms > results[test_type]["highest"]["value"]:
                results[test_type]["highest"] = {"environment": environment, "value": avg_rtt_ms}
            
            # Update the lowest RTT if current value is lower
            if avg_rtt_ms < results[test_type]["lowest"]["value"]:
                results[test_type]["lowest"] = {"environment": environment, "value": avg_rtt_ms}
    
    return results

def generate_latex_table(results):
    # LaTeX table header
    latex_table = (
        "\\begin{table}[H]\n"
        "    \\centering\n"
        "    \\caption{Highest and Lowest Average RTTs across Environments for Each Test Type}\n"
        "    \\label{tab:extreme_rtt}\n"
        "    \\resizebox{\\textwidth}{!}{\n"
        "        \\renewcommand{\\arraystretch}{1.3}\n"
        "        \\setlength{\\tabcolsep}{10pt}\n"
        "        \\begin{tabular}{|l|c|c|}\n"
        "            \\hline\n"
        "            \\rowcolor{black!90}\n"
        "            \\textcolor{white}{\\textbf{Test Type}} & "
        "\\textcolor{white}{\\textbf{Environment (Highest RTT)}} & "
        "\\textcolor{white}{\\textbf{Environment (Lowest RTT)}} \\\\\n"
        "            \\hline\n"
    )
    
    # Add rows for each test type
    row_color = "\\rowcolor{gray!20}"
    for i, (test_type, extremes) in enumerate(results.items()):
        highest_env = extremes["highest"]["environment"]
        highest_rtt = extremes["highest"]["value"]
        lowest_env = extremes["lowest"]["environment"]
        lowest_rtt = extremes["lowest"]["value"]
        
        # Add row to the table
        latex_table += (
            f"            {row_color if i % 2 == 0 else ''} "
            f"{test_type} & {highest_env} ({highest_rtt:.3f} ms) & {lowest_env} ({lowest_rtt:.3f} ms) \\\\\n"
            "            \\hline\n"
        )
    
    # LaTeX table footer
    latex_table += (
        "        \\end{tabular}\n"
        "    }\n"
        "\\end{table}"
    )
    
    return latex_table

# Find the highest and lowest average RTTs
extreme_rtts = find_extreme_rtts(data)

# Generate the LaTeX table
latex_output = generate_latex_table(extreme_rtts)

# Output the LaTeX table
print(latex_output)
