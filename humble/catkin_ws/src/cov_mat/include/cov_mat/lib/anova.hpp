#ifndef ANOVACALCULATOR_H
#define ANOVACALCULATOR_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <boost/math/distributions/fisher_f.hpp>

// Alias for 3D vector of floats
using Float3DVectorMatrix = std::vector<std::vector<std::vector<float>>>;
using Float3DVectorMatrixPtr = std::shared_ptr<Float3DVectorMatrix>;

// Anova abstract class
class AnovaBase
{
public:
    virtual bool main() = 0;
    virtual void set_verbose(bool verbose) { verbose_ = verbose; };
    virtual bool get_verbose() { return verbose_; };
    virtual void set_verbose_dev(bool verbose_dev) { verbose_dev_ = verbose_dev; };
    virtual bool get_verbose_dev() { return verbose_dev_; };
    virtual void set_alpha(double alpha) { alpha_ = alpha; };
    virtual double get_alpha() { return alpha_; };

    virtual float getF_statisticGroups() { return f_statisticGroups_; }
    virtual float getF_statisticBlocks() { return f_statisticBlocks_; }
    virtual float getF_statisticBlocksGroups() { return f_statisticBlocksGroups_; }

    virtual float getF_criticalGroups() { return f_criticalGroups_; }             // F-critical value for columns
    virtual float getF_criticalBlocks() { return f_criticalBlocks_; }             // F-critical value for rows
    virtual float getF_criticalBlocksGroups() { return f_criticalBlocksGroups_; } // F-critical value for interaction

    virtual double getP_valueGroups() { return p_valueGroups_; }             // P-value for groups/columns
    virtual double getP_valueBlocks() { return p_valueBlocks_; }             // P-value for blocks/rows
    virtual double getP_valueBlocksGroups() { return p_valueBlocksGroups_; } // P-value for interaction

protected:
    bool verbose_ = false;
    bool verbose_dev_ = false;
    double alpha_ = 0.05; // Significance level

    int n_groups_ = 0; // Number of groups/columns
    int n_blocks_ = 0; // Number of blocks/rows

    float overallMean_ = 0.0; // Overall mean
    int n_elements_ = 0;      // Total number of elements

    float SST_;  // Sum of squares total error
    float SSC_;  // Sum of squares groups/columns error
    float SSB_;  // Sum of squares blocks error
    float SSW_;  // Sum of squares within cells error
    float SSE_;  // Sum of squares remaining error
    float SSAB_; // Sum of squares interaction error

    int SST_df_;  // Degrees of freedom for SST
    int SSC_df_;  // Degrees of freedom for SSC
    int SSB_df_;  // Degrees of freedom for SSB
    int SSW_df_;  // Degrees of freedom for SSW
    int SSE_df_;  // Degrees of freedom for SSE
    int SSAB_df_; // Degrees of freedom for SSAB

    float MST_;  // Mean Sum Square Total
    float MSC_;  // Mean Sum Square columns
    float MSB_;  // Mean Square Block
    float MSW_;  // Mean Square Within cells
    float MSE_;  // Mean Square Error
    float MSAB_; // Mean Square Interaction

    float f_statisticGroups_;       // F-statistic between columns/groups (F-ratio)
    float f_statisticBlocks_;       // F-statistic between rows/blocks (F-ratio)
    float f_statisticBlocksGroups_; // F-statistic between interaction (F-ratio)

    double f_criticalGroups_;       // F-critical value for groups/columns
    double f_criticalBlocks_;       // F-critical value for blocks/rows
    double f_criticalBlocksGroups_; // F-critical value for interaction

    double p_valueGroups_;       // P-value for groups/columns
    double p_valueBlocks_;       // P-value for blocks/rows
    double p_valueBlocksGroups_; // P-value for interaction

    virtual double getfDistributionCriticalValue(double alpha, int df_numerator, int df_denominator);
    virtual double calculatePValue(double F, int numerator_df, int denominator_df);

    virtual void setupAnovaParams() = 0;

    virtual float computeF_statistic() = 0; // Compute F-statistic values (F-ratio)
    virtual float computeP_value() = 0;     // Compute P-values
    virtual float computeF_critical() = 0;  // Compute F-critical values
};

class OneWayAnovaCalculator : public AnovaBase
{

public:
    /**
     * @brief Construct a new OneWayAnovaCalculator object
     */
    OneWayAnovaCalculator(const std::vector<std::vector<float>> &data);

    /**
     * @brief Destroy the OneWayAnovaCalculator object
     */
    ~OneWayAnovaCalculator();

    /**
     * @brief main function
     *
     * @return Returns true if F-statistic is greater than F-critical value
     */
    bool main() override;

private:
    std::vector<std::vector<float>> data_;

    Eigen::VectorXd within_group_mean_;

    void setupAnovaParams() override;

    float computeSST(); // Compute SST - (overall) sum of squares total
    float computeSSE(); // Compute SSE - (within/overall) sum of squares error within class
    float computeSSC(); // Compute SSC - (between/overall) sum of squares between class

    float computeF_statistic() override; // Compute F-statistic (F-ratio)
    float computeP_value() override;     // Compute P-value
    float computeF_critical() override;  // Compute F-critical value
};

class TwoWaysAnovaCalcNoRep : public AnovaBase
{
    // This is also called Randomized Block Design

public:
    /**
     * @brief Construct a new TwoWayAnovaCalculatorWithoutReplication object
     */
    TwoWaysAnovaCalcNoRep(const std::vector<std::vector<float>> &data);

    /**
     * @brief Destroy the TwoWayAnovaCalculatorWithoutReplication object
     */
    ~TwoWaysAnovaCalcNoRep();

    /**
     * @brief main function
     *
     * @return Returns true if F-statistic is greater than F-critical value
     */
    bool main() override;

private:
    std::vector<std::vector<float>> data_;

    Eigen::VectorXd within_group_mean_;
    Eigen::VectorXd within_block_mean_;

    void setupAnovaParams() override;

    float computeSST();                  // Compute SST - (overall) sum of squares total
    float computeSSC();                  // Compute SSC - (between/overall) sum of squares between class
    float computeSSE();                  // Compute SSE - (remain error/overall) sum of squares error within class
    float computeSSB();                  // Compute SSB - (block/overall) sum of squares error within class
    float computeF_statistic() override; // Compute F-statistic (F-ratio)
    float computeP_value() override;     // Compute P-values
    float computeF_critical() override;  // Compute F-critical values
};

class TwoWaysAnovaCalcWithRep : public AnovaBase
{

public:
    /**
     * @brief Construct a new TwoWayAnovaCalculatorWithReplication object
     */
    TwoWaysAnovaCalcWithRep(const Float3DVectorMatrixPtr &data);

    /**
     * @brief Destroy the TwoWayAnovaCalculatorWithReplication object
     */
    ~TwoWaysAnovaCalcWithRep();

    /**
     * @brief main function
     *
     * @return Returns true if F-statistic is greater than F-critical value
     */
    bool main() override;

private:
    Float3DVectorMatrixPtr data_;

    Eigen::VectorXd within_group_mean_;
    Eigen::VectorXd within_block_mean_;
    Eigen::MatrixXd cell_mean_;
    Eigen::MatrixXd n_elements_byCells_;

    void setupAnovaParams() override;

    float computeSST();  // Compute SST - (overall) sum of squares total
    float computeSSC();  // Compute SSC - (between/overall) sum of squares between groups
    float computeSSB();  // Compute SSB - (block/overall) sum of squares error between blocks
    float computeSSW();  // Compute SSW - (within/overall) sum of squares
    float computeSSAB(); // Compute SSAB - (interaction/overall) sum of squares
    float computeSSE();  // Compute SSE - (remain error/overall) sum of squares error remain error

    float computeF_statistic() override; // Compute F-statistic (F-ratio)
    float computeP_value() override;     // Compute P-values
    float computeF_critical() override;  // Compute F-critical values
};

#endif // ANOVACALCULATOR_H
