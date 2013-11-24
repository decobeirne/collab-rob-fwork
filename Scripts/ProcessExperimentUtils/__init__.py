"""Extract data from results files, analyse and print to csv files for generating graphs."""

__author__ = 'Declan O\'Beirne'

from ProcessExperimentUtils.extractResults import extractResultsFromExperiments
from ProcessExperimentUtils.analyseResults import analyseResultsDictList
from ProcessExperimentUtils.printResults import printResultsToCsvFiles