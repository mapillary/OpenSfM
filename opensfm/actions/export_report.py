from opensfm import report
from opensfm.dataset import DataSet


def run_dataset(data: DataSet):
    """Export a nice report based on previously generated statistics

    Args:
        data: dataset object

    """
    pdf_report = report.Report(data)
    pdf_report.generate_report()
    pdf_report.save_report("report.pdf")
