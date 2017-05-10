/**
 * @class LibIntentions
 * @author Li Shu <tongjilishu@163.com>
 * @date 13/04/16
 * @file LibIntentions.h
 * @brief Declears some intentions each role may have, mainly used in team communication.
 */

#pragma once

  enum class KeeperIntention 
  {
    NONE,           /**< just stand, eg. the ball is on the opponent field  */
    STAND_TO_KEEP,  /**< the process or state that keeper face the ball and stand at the right position */
    SPREAD_LEG,     /**< the ball is dangerous to goal */
    GO_TO_BALL,     /**< go to ball (Note: the defener and supporter should walk away or try to prevent others from steeling the ball) */
    KICK_BALL,      /**< kick the ball (Note: the same as GO_TO_BALL)*/
    SEARCH_BALL,    /**< if the whole team can't see a ball, all should search ball */
  };
  
  enum class SupporterIntention
  {
      NONE,                 /**< just stand, less used in supporter  */
      WALK_TO_DEFEND_LINE,  /**< walk to line x = -3200  */
      WALK_TO_HALF_FIELD,   /**< walk to line x = -2000  */
      WALK_TO_CENTER_CIRCLE,/**< walk to line x = 0  */
      STAND_TO_DEFEND,      /**< face to ball and ready to defend  */
      GO_TO_BALL,           /**< go to the ball (Note: defender should walk away or tyr to prevent others from steeling the ball)  */
      KICK_BALL,            /**< kick the ball (Note: the same as GO_TO_BALL)  */
      SEARCH_BALL,          /**< if the whole team can't see a ball, all should search ball */
  };
  
  enum class DefenderIntention
  {
      NONE,                 /**< just stand, eg. the ball is on the opponent field */
      WALK_TO_DEFEND_LINE,  /**< walk to line x = -3200  */
      STAND_TO_DEFEND,      /**< face to ball and ready to defend  */
      GO_TO_BALL,           /**< go to the ball (Note: defender should walk away or tyr to prevent others from steeling the ball)  */
      KICK_BALL,            /**< kick the ball (Note: the same as GO_TO_BALL)  */
      SEARCH_BALL,          /**< if the whole team can't see a ball, all should search ball */
  };
  

