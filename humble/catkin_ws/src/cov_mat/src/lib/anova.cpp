#include <cov_mat/lib/anova.hpp>

double AnovaBase::getfDistributionCriticalValue(double alpha, int df_numerator, int df_denominator)
{
    if (alpha <= 0.0 || alpha >= 1.0 || df_numerator <= 0 || df_denominator <= 0)
    {
        std::cerr << "Invalid input parameters." << std::endl;
        return -1.0; // Invalid input
    }

    boost::math::fisher_f_distribution<double> fDist(df_numerator, df_denominator);

    // Calculate the critical value using the quantile function
    double f_critical = boost::math::quantile(boost::math::complement(fDist, alpha));

    return f_critical;
}

double AnovaBase::calculatePValue(double F, int numerator_df, int denominator_df)
{
    if (F < 0 || numerator_df <= 0 || denominator_df <= 0)
    {
        std::cerr << "Invalid input values. F, numerator_df, and denominator_df must be positive." << std::endl;
        return -1.0; // Indicate an error
    }

    // Define the F-distribution
    boost::math::fisher_f dist(numerator_df, denominator_df);

    // Calculate the p-value
    double p_value = 1 - boost::math::cdf(dist, F); // 1 - cdf(F)

    return p_value; // Return the final p-value
}

OneWayAnovaCalculator::OneWayAnovaCalculator(const std::vector<std::vector<float>> &data) : data_(data)
{
    n_groups_ = data.size();
}

OneWayAnovaCalculator::~OneWayAnovaCalculator() {}

void OneWayAnovaCalculator::setupAnovaParams()
{
    within_group_mean_ = Eigen::VectorXd::Zero(n_groups_);

    overallMean_ = 0;
    n_elements_ = 0;

    // Compute Average of each group and overall Mean
    for (size_t groupId = 0; groupId < size_t(n_groups_); groupId++)
    {
        float sum = 0;
        for (size_t i = 0; i < data_[groupId].size(); i++)
        {
            n_elements_ = n_elements_ + 1;
            sum += data_[groupId][i];
            overallMean_ += data_[groupId][i];
        }
        within_group_mean_(groupId) = sum / data_[groupId].size();
    }

    overallMean_ /= n_elements_;
}

float OneWayAnovaCalculator::computeSST()
{
    // Compute SST - (overall) sum of squares total
    SST_ = 0;
    SST_df_ = n_elements_ - 1;
    for (size_t groupId = 0; groupId < size_t(n_groups_); groupId++)
    {
        for (size_t i = 0; i < data_[groupId].size(); i++)
        {
            SST_ += pow(data_[groupId][i] - overallMean_, 2);
        }
    }
    if (verbose_dev_)
    {
        std::cout << "SST: " << SST_ << std::endl;
        std::cout << "SST_df: " << SST_df_ << std::endl;
    }

    return SST_;
}

float OneWayAnovaCalculator::computeSSE()
{
    // Compute SSE - (within/overall) sum of squares error within class
    SSE_ = 0;
    SSE_df_ = n_elements_ - n_groups_;
    for (size_t groupId = 0; groupId < size_t(n_groups_); groupId++)
    {
        for (size_t i = 0; i < data_[groupId].size(); i++)
        {
            SSE_ += pow(data_[groupId][i] - within_group_mean_(groupId), 2);
        }
    }
    if (verbose_dev_)
    {
        std::cout << "SSE: " << SSE_ << std::endl;
        std::cout << "SSE_df: " << SSE_df_ << std::endl;
    }

    return SSE_;
}

float OneWayAnovaCalculator::computeSSC()
{
    // Compute SSC - (between/overall) sum of squares between groups
    SSC_ = 0;
    SSC_df_ = n_groups_ - 1;
    for (size_t groupId = 0; groupId < size_t(n_groups_); groupId++)
    {
        SSC_ += pow(within_group_mean_(groupId) - overallMean_, 2) * data_[groupId].size();
    }
    if (verbose_dev_)
    {
        std::cout << "SSC: " << SSC_ << std::endl;
        std::cout << "SSC_df: " << SSC_df_ << std::endl;
    }

    return SSC_;
}

float OneWayAnovaCalculator::computeF_statistic()
{
    // Compute MSC, MSE, F_statistic
    MSC_ = SSC_ / SSC_df_;
    MSE_ = SSE_ / SSE_df_;
    f_statisticGroups_ = MSC_ / MSE_;

    if (verbose_dev_)
    {
        std::cout << "MSC: " << MSC_ << std::endl;
        std::cout << "MSE: " << MSE_ << std::endl;
        std::cout << "F-statistic: " << f_statisticGroups_ << std::endl;
    }

    return f_statisticGroups_;
}

float OneWayAnovaCalculator::computeP_value()
{
    // Compute P-value
    p_valueGroups_ = calculatePValue(f_statisticGroups_, SSC_df_, SSE_df_);

    if (verbose_dev_)
    {
        std::cout << "P-value: " << p_valueGroups_ << std::endl;
    }

    return p_valueGroups_;
}

float OneWayAnovaCalculator::computeF_critical()
{
    // Compute F-critical value
    f_criticalGroups_ = getfDistributionCriticalValue(alpha_, SSC_df_, SSE_df_);

    if (verbose_dev_)
    {
        std::cout << "F-critical Groups value: " << f_criticalGroups_ << std::endl;
    }

    return f_criticalGroups_;
}

bool OneWayAnovaCalculator::main()
{

    // Settup ANOVA parameters
    setupAnovaParams();

    if (verbose_dev_)
    {
        std::cout << "Overall Mean: " << overallMean_ << std::endl;
        std::cout << "n_elements: " << n_elements_ << std::endl;
        std::cout << "n_classes: " << n_groups_ << std::endl;
    }

    // Compute SST, SSE, SSC
    computeSST();
    computeSSE();
    computeSSC();

    // Compute F-statistic
    computeF_statistic();

    // Compute P-value
    computeP_value();

    // Compute F-critical value
    f_criticalGroups_ = getfDistributionCriticalValue(alpha_, SSC_df_, SSE_df_);

    // Compare F-statistic and F-critical value
    if (f_statisticGroups_ > f_criticalGroups_)
    {
        if (verbose_dev_)
            std::cout << "F-statistic is greater than F-critical value" << std::endl;
        return true;
    }
    else
    {
        if (verbose_dev_)
            std::cout << "F-statistic is less than F-critical value" << std::endl;
        return false;
    }
}

TwoWaysAnovaCalcNoRep::TwoWaysAnovaCalcNoRep(const std::vector<std::vector<float>> &data) : data_(data)
{
    n_groups_ = data_.size();
}

TwoWaysAnovaCalcNoRep::~TwoWaysAnovaCalcNoRep() {}

void TwoWaysAnovaCalcNoRep::setupAnovaParams()
{
    within_group_mean_ = Eigen::VectorXd::Zero(n_groups_);
    n_blocks_ = data_[0].size();

    overallMean_ = 0;
    n_elements_ = 0;

    // Compute Average of each group and overall Mean
    for (size_t groupId = 0; groupId < size_t(n_groups_); groupId++)
    {
        float sum = 0;
        for (size_t i = 0; i < data_[groupId].size(); i++)
        {
            n_elements_ = n_elements_ + 1;
            sum += data_[groupId][i];
            overallMean_ += data_[groupId][i];
        }
        within_group_mean_(groupId) = sum / data_[groupId].size();
    }

    // Compute Average of each block
    within_block_mean_ = Eigen::VectorXd::Zero(n_blocks_);
    for (size_t blockId = 0; blockId < size_t(n_blocks_); blockId++)
    {
        float sum = 0;
        for (size_t i = 0; i < data_.size(); i++)
        {
            sum += data_[i][blockId];
        }
        within_block_mean_(blockId) = sum / data_.size();
    }

    overallMean_ /= n_elements_;
}

float TwoWaysAnovaCalcNoRep::computeSST()
{
    // Compute SST - (overall) sum of squares total
    SST_ = 0;
    SST_df_ = n_elements_ - 1;
    for (size_t groupId = 0; groupId < size_t(n_groups_); groupId++)
    {
        for (size_t i = 0; i < data_[groupId].size(); i++)
        {
            SST_ += pow(data_[groupId][i] - overallMean_, 2);
        }
    }
    if (verbose_dev_)
    {
        std::cout << "SST: " << SST_ << std::endl;
        std::cout << "SST_df: " << SST_df_ << std::endl;
    }

    return SST_;
}

float TwoWaysAnovaCalcNoRep::computeSSC()
{
    // Compute SSC - (between/overall) sum of squares between groups
    SSC_ = 0;
    SSC_df_ = n_groups_ - 1;
    for (size_t groupId = 0; groupId < size_t(n_groups_); groupId++)
    {
        SSC_ += pow(within_group_mean_(groupId) - overallMean_, 2) * data_[groupId].size();
    }
    if (verbose_dev_)
    {
        std::cout << "SSC: " << SSC_ << std::endl;
        std::cout << "SSC_df: " << SSC_df_ << std::endl;
    }

    return SSC_;
}

float TwoWaysAnovaCalcNoRep::computeSSB()
{
    // Compute SSB - (block/overall) sum of squares error within blocks
    SSB_ = 0;
    SSB_df_ = n_blocks_ - 1;
    for (size_t blockId = 0; blockId < size_t(n_blocks_); blockId++)
    {
        SSB_ += pow(within_block_mean_(blockId) - overallMean_, 2) * n_groups_;
    }
    if (verbose_dev_)
    {
        std::cout << "SSB: " << SSB_ << std::endl;
        std::cout << "SSB_df: " << SSB_df_ << std::endl;
    }

    return SSB_;
}

float TwoWaysAnovaCalcNoRep::computeSSE()
{
    // Compute SSE - (within/overall) sum of squares error within class
    SSE_ = SST_ - SSC_ - SSB_;
    SSE_df_ = (n_blocks_ - 1) * (n_groups_ - 1);

    if (verbose_dev_)
    {
        std::cout << "SSE: " << SSE_ << std::endl;
        std::cout << "SSE_df: " << SSE_df_ << std::endl;
    }

    return SSE_;
}

float TwoWaysAnovaCalcNoRep::computeF_statistic()
{
    // Compute MSC, MSE, F_statistic
    MST_ = SST_ / SST_df_;
    MSC_ = SSC_ / SSC_df_;
    MSB_ = SSB_ / SSB_df_;
    MSE_ = SSE_ / SSE_df_;

    // f_statisticGroups_
    f_statisticGroups_ = MSC_ / MSE_;

    f_statisticBlocks_ = MSB_ / MSE_;

    if (verbose_dev_)
    {
        std::cout << "MST: " << MST_ << std::endl;
        std::cout << "MSC: " << MSC_ << std::endl;
        std::cout << "MSB: " << MSB_ << std::endl;
        std::cout << "MSE: " << MSE_ << std::endl;
        std::cout << "F-statistic Groups: " << f_statisticGroups_ << std::endl;
        std::cout << "F-statistic Blocks: " << f_statisticBlocks_ << std::endl;
    }

    return f_statisticGroups_;
}

float TwoWaysAnovaCalcNoRep::computeP_value()
{
    // Compute P-value
    p_valueGroups_ = calculatePValue(f_statisticGroups_, SSC_df_, SSE_df_);
    p_valueBlocks_ = calculatePValue(f_statisticBlocks_, SSB_df_, SSE_df_);

    if (verbose_dev_)
    {
        std::cout << "P-value Groups: " << p_valueGroups_ << std::endl;
        std::cout << "P-value Blocks: " << p_valueBlocks_ << std::endl;
    }

    return p_valueGroups_;
}

float TwoWaysAnovaCalcNoRep::computeF_critical()
{
    // Compute F-critical value
    f_criticalGroups_ = getfDistributionCriticalValue(alpha_, SSC_df_, SSE_df_);
    f_criticalBlocks_ = getfDistributionCriticalValue(alpha_, SSB_df_, SSE_df_);

    if (verbose_dev_)
    {
        std::cout << "F-critical Groups value: " << f_criticalGroups_ << std::endl;
        std::cout << "F-critical Blocks value: " << f_criticalBlocks_ << std::endl;
    }

    return f_criticalGroups_;
}

bool TwoWaysAnovaCalcNoRep::main()
{
    // Settup ANOVA parameters
    setupAnovaParams();

    if (verbose_dev_)
    {
        std::cout << "Overall Mean: " << overallMean_ << std::endl;
        std::cout << "n_elements: " << n_elements_ << std::endl;
        std::cout << "n_classes: " << n_groups_ << std::endl;
        std::cout << "n_blocks_: " << n_blocks_ << std::endl;
    }

    // Compute SST, SSC, SSB, SSE
    computeSST();
    computeSSC();
    computeSSB();
    computeSSE();

    // Compute F-statistic
    computeF_statistic();

    // Compute F-critical value
    f_criticalGroups_ = getfDistributionCriticalValue(alpha_, SSC_df_, SSE_df_);

    // Compare F-statistic and F-critical value
    if (f_statisticGroups_ > f_criticalGroups_)
    {
        if (verbose_dev_)
            std::cout << "F-statistic is greater than F-critical value" << std::endl;
        return true;
    }
    else
    {
        if (verbose_dev_)
            std::cout << "F-statistic is less than F-critical value" << std::endl;
        return false;
    }
}

TwoWaysAnovaCalcWithRep::TwoWaysAnovaCalcWithRep(const Float3DVectorMatrixPtr &data) : data_(data)
{
    n_groups_ = data_->size();
}

TwoWaysAnovaCalcWithRep::~TwoWaysAnovaCalcWithRep() {}

void TwoWaysAnovaCalcWithRep::setupAnovaParams()
{
    n_blocks_ = data_->at(0).size();
    cell_mean_ = Eigen::MatrixXd::Zero(n_groups_, n_blocks_);
    n_elements_byCells_ = Eigen::MatrixXd::Zero(n_groups_, n_blocks_);

    overallMean_ = 0;
    n_elements_ = 0;

    // Compute mean of each cell and overall Mean
    for (size_t groupId = 0; groupId < size_t(n_groups_); groupId++)
    {
        for (size_t blockId = 0; blockId < size_t(n_blocks_); blockId++)
        {
            float sum = 0;
            float element_byCell_tmp = 0;
            for (size_t i = 0; i < data_->at(groupId)[blockId].size(); i++)
            {
                n_elements_ = n_elements_ + 1;
                element_byCell_tmp = element_byCell_tmp + 1;
                sum += data_->at(groupId)[blockId][i];
                overallMean_ += data_->at(groupId)[blockId][i];
            }
            cell_mean_(groupId, blockId) = sum / data_->at(groupId)[blockId].size();
            n_elements_byCells_(groupId, blockId) = element_byCell_tmp;
        }
    }

    overallMean_ /= n_elements_;

    // Compute mean of each group
    within_group_mean_ = Eigen::VectorXd::Zero(n_groups_);
    for (size_t groupId = 0; groupId < size_t(n_groups_); groupId++)
    {
        float sum = 0;
        for (size_t blockId = 0; blockId < size_t(n_blocks_); blockId++)
        {
            sum += cell_mean_(groupId, blockId);
        }
        within_group_mean_(groupId) = sum / n_blocks_;
    }

    // Compute mean of each block
    within_block_mean_ = Eigen::VectorXd::Zero(n_blocks_);
    for (size_t blockId = 0; blockId < size_t(n_blocks_); blockId++)
    {
        float sum = 0;
        for (size_t groupId = 0; groupId < size_t(n_groups_); groupId++)
        {
            sum += cell_mean_(groupId, blockId);
        }
        within_block_mean_(blockId) = sum / n_groups_;
    }
}

float TwoWaysAnovaCalcWithRep::computeSST()
{
    // Compute SST - (overall) sum of squares total
    SST_ = 0;
    SST_df_ = n_elements_ - 1;
    for (size_t groupId = 0; groupId < size_t(n_groups_); groupId++)
    {
        for (size_t blockId = 0; blockId < data_->at(blockId).size(); blockId++)
        {
            for (size_t i = 0; i < data_->at(groupId)[blockId].size(); i++)
            {
                SST_ += pow(data_->at(groupId)[blockId][i] - overallMean_, 2);
            }
        }
    }
    if (verbose_dev_)
    {
        std::cout << "\nSST: " << SST_ << std::endl;
        std::cout << "SST_df: " << SST_df_ << std::endl;
    }

    return SST_;
}

float TwoWaysAnovaCalcWithRep::computeSSC()
{
    // Compute SSC - (between/overall) sum of squares between groups
    SSC_ = 0;
    SSC_df_ = n_groups_ - 1;
    for (size_t groupId = 0; groupId < size_t(n_groups_); groupId++)
    {
        SSC_ += pow(within_group_mean_(groupId) - overallMean_, 2) * data_->at(groupId).size() * n_elements_byCells_(groupId, 0);
    }
    if (verbose_dev_)
    {
        std::cout << "\nSSC: " << SSC_ << std::endl;
        std::cout << "SSC_df: " << SSC_df_ << std::endl;
    }

    return SSC_;
}

float TwoWaysAnovaCalcWithRep::computeSSB()
{
    // Compute SSB - (block/overall) sum of squares error within blocks
    SSB_ = 0;
    SSB_df_ = n_blocks_ - 1;
    for (size_t blockId = 0; blockId < size_t(n_blocks_); blockId++)
    {
        SSB_ += pow(within_block_mean_(blockId) - overallMean_, 2) * n_groups_ * n_elements_byCells_(blockId, 0);
    }
    if (verbose_dev_)
    {
        std::cout << "\nSSB: " << SSB_ << std::endl;
        std::cout << "SSB_df: " << SSB_df_ << std::endl;
    }

    return SSB_;
}

float TwoWaysAnovaCalcWithRep::computeSSW()
{
    // Compute SSW - (within/overall) sum of squares error within class
    SSW_ = 0;
    SSW_df_ = n_elements_ - (n_groups_ * n_blocks_);
    for (size_t groupId = 0; groupId < size_t(n_groups_); groupId++)
    {
        for (size_t blockId = 0; blockId < size_t(n_blocks_); blockId++)
        {
            for (size_t i = 0; i < data_->at(groupId)[blockId].size(); i++)
            {
                SSW_ += pow(data_->at(groupId)[blockId][i] - cell_mean_(groupId, blockId), 2);
            }
        }
    }
    if (verbose_dev_)
    {
        std::cout << "\nSSW: " << SSW_ << std::endl;
        std::cout << "SSW_df: " << SSW_df_ << std::endl;
    }

    return SSW_;
}

float TwoWaysAnovaCalcWithRep::computeSSAB()
{
    // Compute SSAB - (interaction) sum of squares error crossing groups and blocks
    SSAB_ = 0;
    SSAB_df_ = (n_groups_ - 1) * (n_blocks_ - 1);

    for (size_t groupId = 0; groupId < size_t(n_groups_); groupId++)
    {
        for (size_t blockId = 0; blockId < size_t(n_blocks_); blockId++)
        {
            SSAB_ += n_elements_byCells_(groupId, blockId) * pow(cell_mean_(groupId, blockId) - within_group_mean_(groupId) - within_block_mean_(blockId) + overallMean_, 2);
        }
    }

    if (verbose_dev_)
    {
        std::cout << "\nSSAB: " << SSAB_ << std::endl;
        std::cout << "SSAB_df: " << SSAB_df_ << std::endl;
    }

    return SSAB_;
}

float TwoWaysAnovaCalcWithRep::computeSSE()
{
    // Compute SSE - (within/overall) sum of squares error within class
    SSE_ = SSW_;
    SSE_df_ = SSW_df_;

    if (verbose_dev_)
    {
        std::cout << "\nSSE: " << SSE_ << std::endl;
        std::cout << "SSE_df: " << SSE_df_ << std::endl;
    }

    return SSE_;
}

float TwoWaysAnovaCalcWithRep::computeF_statistic()
{
    // Compute MSC, MSE, F_statistic
    MST_ = SST_ / SST_df_;
    MSC_ = SSC_ / SSC_df_;
    MSB_ = SSB_ / SSB_df_;
    MSE_ = SSE_ / SSE_df_;
    MSW_ = SSW_ / SSW_df_;
    MSAB_ = SSAB_ / SSAB_df_;

    // f_statisticGroups_
    f_statisticGroups_ = MSC_ / MSE_;

    f_statisticBlocks_ = MSB_ / MSE_;

    f_statisticBlocksGroups_ = MSAB_ / MSE_;

    if (verbose_dev_)
    {
        std::cout << "\nMST: " << MST_ << std::endl;
        std::cout << "MSC: " << MSC_ << std::endl;
        std::cout << "MSB: " << MSB_ << std::endl;
        std::cout << "MSE: " << MSE_ << std::endl;
        std::cout << "MSW: " << MSW_ << std::endl;
        std::cout << "MSAB: " << MSAB_ << std::endl;
        std::cout << "\nF-statistic Groups: " << f_statisticGroups_ << std::endl;
        std::cout << "F-statistic Blocks: " << f_statisticBlocks_ << std::endl;
        std::cout << "F-statistic Interaction: " << f_statisticBlocksGroups_ << std::endl;
    }

    return f_statisticGroups_;
}

float TwoWaysAnovaCalcWithRep::computeP_value()
{
    // Compute P-values
    p_valueGroups_ = calculatePValue(f_statisticGroups_, SSC_df_, SSE_df_);
    p_valueBlocks_ = calculatePValue(f_statisticBlocks_, SSB_df_, SSE_df_);
    p_valueBlocksGroups_ = calculatePValue(f_statisticBlocksGroups_, SSAB_df_, SSE_df_);

    if (verbose_dev_)
    {
        std::cout << "\nP-value Groups: " << p_valueGroups_ << std::endl;
        std::cout << "P-value Blocks: " << p_valueBlocks_ << std::endl;
        std::cout << "P-value Interaction: " << p_valueBlocksGroups_ << std::endl;
    }

    return p_valueGroups_;
}

float TwoWaysAnovaCalcWithRep::computeF_critical()
{
    // Compute F-critical value
    f_criticalGroups_ = getfDistributionCriticalValue(alpha_, SSC_df_, SSE_df_);
    f_criticalBlocks_ = getfDistributionCriticalValue(alpha_, SSB_df_, SSE_df_);
    f_criticalBlocksGroups_ = getfDistributionCriticalValue(alpha_, SSAB_df_, SSE_df_);

    if (verbose_dev_)
    {
        std::cout << "\nF-critical Groups value: " << f_criticalGroups_ << std::endl;
        std::cout << "F-critical Blocks value: " << f_criticalBlocks_ << std::endl;
        std::cout << "F-critical Interaction value: " << f_criticalBlocksGroups_ << std::endl;
    }

    return f_criticalGroups_;
}

bool TwoWaysAnovaCalcWithRep::main()
{
    setupAnovaParams();

    if (verbose_dev_)
    {
        std::cout << "\nOverall Mean: " << overallMean_ << std::endl;
        std::cout << "n_elements: " << n_elements_ << std::endl;
        std::cout << "n_classes: " << n_groups_ << std::endl;
        std::cout << "n_blocks_: " << n_blocks_ << std::endl;
    }

    // Compute SST, SSC, SSB, SSW, SSAB, SSE
    computeSST();
    computeSSC();
    computeSSB();
    computeSSW();
    computeSSAB();
    computeSSE();

    // Compute F-statistic
    computeF_statistic();

    // Compute P-value
    computeP_value();

    // Compute F-critical values
    computeF_critical();

    return true;
}