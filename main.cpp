#include <WITCH/WITCH.h>

#include <fan/graphics/opengl/3D/objects/fms.h>
#include <iomanip>

#include <stdint.h>

#include <cctype>

struct bone_names_default_t : __dme_inherit(bone_names_default_t, std::vector<std::string>){
  #define d(name, ...) __dme(name) = {{#name, ##__VA_ARGS__}};
  d(hips, "Torso");
  d(right_up_leg, "Upper_Leg_R", "RightUpLeg", "Upper_Leg.R", "R_UpperLeg");
  d(lower_leg_r, "Lower_Leg.R", "Right_knee", "R_LowerLeg");
  d(right_foot, "Foot_R", "RightFoot", "Foot.R", "Right_ankle", "R_Foot");
  d(right_toe_base, "RightToeBase", "Right_toe", "R_ToeBase");
  d(right_toe_end);
  d(spine);
  d(spine1, "Chest", "UpperChest");
  d(spine2);
  d(neck);
  d(head);
  d(head_top_end);
  d(left_shoulder, "Upper_Arm_L", "L_Shoulder");
  d(left_arm, "L_UpperArm");
  d(left_fore_arm, "Lower_Arm_L", "Left_elbow", "L_LowerArm");
  d(left_hand, "Hand_L", "Left_wrist", "L_Hand");
  d(left_hand_thumb1, "Thumb0_L");
  d(left_hand_thumb2, "Thumb1_L");
  d(left_hand_thumb3, "Thumb2_L");
  d(left_hand_thumb4, "Thumb2Tip_L");
  d(left_hand_index1, "IndexFinger1_L");
  d(left_hand_index2, "IndexFinger2_L");
  d(left_hand_index3, "IndexFinger3_L");
  d(left_hand_index4, "IndexFinger3Tip_L");
  d(left_hand_middle1, "MiddleFinger1_L");
  d(left_hand_middle2, "MiddleFinger2_L");
  d(left_hand_middle3, "MiddleFinger3_L");
  d(left_hand_middle4, "MiddleFinger3Tip_L");
  d(left_hand_ring1, "RingFinger1_L");
  d(left_hand_ring2, "RingFinger2_L");
  d(left_hand_ring3, "RingFinger3_L");
  d(left_hand_ring4, "RingFinger3Tip_L");
  d(left_hand_pinky1, "LittleFinger1_L");
  d(left_hand_pinky2, "LittleFinger2_L");
  d(left_hand_pinky3, "LittleFinger3_L");
  d(left_hand_pinky4, "LittleFinger3Tip_L");
  d(right_shoulder, "Upper_Arm_R", "R_Shoulder");
  d(right_arm, "R_UpperArm");
  d(right_fore_arm, "Lower_Arm_R", "Right_elbow", "R_LowerArm");
  d(right_hand, "Hand_R", "Right_wrist", "R_Hand");
  d(right_hand_thumb1, "Thumb0_R");
  d(right_hand_thumb2, "Thumb1_R");
  d(right_hand_thumb3, "Thumb2_R");
  d(right_hand_thumb4, "Thumb2Tip_R");
  d(right_hand_index1, "IndexFinger1_R");
  d(right_hand_index2, "IndexFinger2_R");
  d(right_hand_index3, "IndexFinger3_R");
  d(right_hand_index4, "IndexFinger3Tip_R");
  d(right_hand_middle1, "MiddleFinger1_R");
  d(right_hand_middle2, "MiddleFinger2_R");
  d(right_hand_middle3, "MiddleFinger3_R");
  d(right_hand_middle4, "MiddleFinger3Tip_R");
  d(right_hand_ring1, "RingFinger1_R");
  d(right_hand_ring2, "RingFinger2_R");
  d(right_hand_ring3, "RingFinger3_R");
  d(right_hand_ring4, "RingFinger3Tip_R");
  d(right_hand_pinky1, "LittleFinger1_R");
  d(right_hand_pinky2, "LittleFinger2_R");
  d(right_hand_pinky3, "LittleFinger3_R");
  d(right_hand_pinky4, "LittleFinger3Tip_R");
  d(left_up_leg, "Upper_Leg_L", "LeftUpLeg", "Upper_Leg.L", "L_UpperLeg");
  d(lower_leg_l, "Lower_Leg.L", "Left_knee", "L_LowerLeg");
  d(left_foot, "Foot_L", "LeftFoot", "Foot.L", "Left_ankle", "L_Foot");
  d(left_toe_base, "LeftToeBase", "Left_toe", "L_ToeBase");
  d(left_toe_end);
  #undef d

  uintptr_t size(){
    return this->GetMemberAmount();
  }
  auto &operator[](uintptr_t i){
    return *this->NA(i);
  }
};

bone_names_default_t bone_names_default;

bone_names_default_t bone_names_anim = bone_names_default;
bone_names_default_t bone_names_model = bone_names_default;

uintptr_t get_bone_name_index(auto &iteration, std::string from){
  uintptr_t longest_name_id = (uintptr_t)-1;
  uintptr_t longest_length = 0;
  uintptr_t shortest_length_from = (uintptr_t)-1;

  for(uintptr_t i = 0; i < iteration.size(); i++){
    for(uintptr_t bni1 = 0; bni1 < iteration[i].size(); bni1++){
      std::string bn = iteration[i][bni1];
      for(uintptr_t ip = 0; ip < from.size(); ip++){
        uintptr_t bni = 0;
        auto nfrom = from.substr(ip);
        uintptr_t nfromi = 0;
        while(1){
          if(bni == bn.size()){
            if(
              (bni > longest_length) ||
              (from.size() < shortest_length_from)
            ){
              longest_name_id = i;
              longest_length = bni;
              shortest_length_from = from.size();
            }
            break;
          }

          if(nfromi == nfrom.size()){
            break;
          }

          if(std::tolower(bn[bni]) == std::tolower(nfrom[nfromi])){
            bni++;
            nfromi++;
            continue;
          }
          else if(bn[bni] == '_'){
            bni++;
          }
          else{
            break;
          }
        }
      }
    }
  }

  return longest_name_id;
}

std::string get_model_bone_name(uintptr_t name_index, auto &model){
  std::string longest_name;
  uintptr_t longest_length = 0;
  uintptr_t shortest_length_from = (uintptr_t)-1;

  model.iterate_bones(*model.root_bone, [&](fan_3d::model::bone_t& bone){
    std::string from = bone.name;

    for(uintptr_t bni1 = 0; bni1 < bone_names_model.NA(name_index)->size(); bni1++){
      std::string bn = (*bone_names_model.NA(name_index))[bni1];
      for(uintptr_t ip = 0; ip < from.size(); ip++){
        uintptr_t bni = 0;
        auto nfrom = from.substr(ip);
        uintptr_t nfromi = 0;
        while(1){
          if(bni == bn.size()){
            if(
              (from.size() < shortest_length_from) ||
              (from.size() == shortest_length_from && bni > longest_length)
            ){
              longest_name = from;
              longest_length = bni;
              shortest_length_from = from.size();
            }
            break;
          }

          if(nfromi == nfrom.size()){
            break;
          }

          if(std::tolower(bn[bni]) == std::tolower(nfrom[nfromi])){
            bni++;
            nfromi++;
            continue;
          }
          else if(bn[bni] == '_'){
            bni++;
          }
          else{
            break;
          }
        }
      }
    }
  });

  return longest_name;
}

void solve_legs(auto &iterator, auto &vector){
  uint8_t left_leg_meaning = (uint8_t)-1;
  std::vector<std::vector<std::string>> left_leg_vector = {
    {"Left_leg"}
  };
  bool left_leg = false;
  iterator.iterate_bones(*iterator.root_bone, [&](fan_3d::model::bone_t& bone){
    if(get_bone_name_index(left_leg_vector, bone.name) == 0){
      if(left_leg){
        /* multiple left leg? */
        assert(0);
      }
      left_leg = true;
    }
  });
  if(left_leg){
    std::vector<std::vector<std::string>> left_down_leg_vector = {
      {"Lower_Leg_L", "Left_knee"}
    };
    iterator.iterate_bones(*iterator.root_bone, [&](fan_3d::model::bone_t& bone){
      if(get_bone_name_index(left_down_leg_vector, bone.name) == 0){
        if(left_leg_meaning != (uint8_t)-1){
          /* multiple left leg meaning? */
          assert(0);
        }
        left_leg_meaning = 0;
      }
    });
    std::vector<std::vector<std::string>> left_up_leg_vector = {
      {"Left_Up_Leg"}
    };
    iterator.iterate_bones(*iterator.root_bone, [&](fan_3d::model::bone_t& bone){
      if(get_bone_name_index(left_up_leg_vector, bone.name) == 0){
        if(left_leg_meaning != (uint8_t)-1){
          /* multiple left leg meaning? */
          assert(0);
        }
        left_leg_meaning = 1;
      }
    });
  }
  if(left_leg_meaning != (uint8_t)-1){
    uintptr_t index;
    if(left_leg_meaning == 0){
      index = get_bone_name_index(bone_names_default, "Left_Up_Leg");
    }
    else if(left_leg_meaning == 1){
      index = get_bone_name_index(bone_names_default, "Lower_Leg_L");
    }
    else{
      index = (uintptr_t)-1;
    }
    if(index == (uintptr_t)-1){
      /* internal error */
      assert(0);
    }

    vector.NA(index)->push_back("Left_leg");
  }

  uint8_t right_leg_meaning = (uint8_t)-1;
  std::vector<std::vector<std::string>> right_leg_vector = {
    {"Right_leg"}
  };
  bool right_leg = false;
  iterator.iterate_bones(*iterator.root_bone, [&](fan_3d::model::bone_t& bone){
    if(get_bone_name_index(right_leg_vector, bone.name) == 0){
      if(right_leg){
        /* multiple right leg? */
        assert(0);
      }
      right_leg = true;
    }
  });
  if(right_leg){
    std::vector<std::vector<std::string>> right_down_leg_vector = {
      {"Lower_Leg_R", "Right_knee"}
    };
    iterator.iterate_bones(*iterator.root_bone, [&](fan_3d::model::bone_t& bone){
      if(get_bone_name_index(right_down_leg_vector, bone.name) == 0){
        if(right_leg_meaning != (uint8_t)-1){
          /* multiple right leg meaning? */
          assert(0);
        }
        right_leg_meaning = 0;
      }
    });
    std::vector<std::vector<std::string>> right_up_leg_vector = {
      {"Right_Up_Leg"}
    };
    iterator.iterate_bones(*iterator.root_bone, [&](fan_3d::model::bone_t& bone){
      if(get_bone_name_index(right_up_leg_vector, bone.name) == 0){
        if(right_leg_meaning != (uint8_t)-1){
          /* multiple right leg meaning? */
          assert(0);
        }
        right_leg_meaning = 1;
      }
    });
  }
  if(right_leg_meaning != (uint8_t)-1){
    uintptr_t index;
    if(right_leg_meaning == 0){
      index = get_bone_name_index(bone_names_default, "Right_Up_Leg");
    }
    else if(right_leg_meaning == 1){
      index = get_bone_name_index(bone_names_default, "Lower_Leg_R");
    }
    else{
      index = (uintptr_t)-1;
    }
    if(index == (uintptr_t)-1){
      /* internal error */
      assert(0);
    }

    vector.NA(index)->push_back("Right_leg");
  }
}

void mapper(fan_3d::model::fms_t& model, fan_3d::model::fms_t& anim){
  solve_legs(anim, bone_names_anim);
  solve_legs(model, bone_names_model);

  anim.iterate_bones(*anim.root_bone, [&](fan_3d::model::bone_t& bone){
    auto bone_name_index = get_bone_name_index(bone_names_anim, bone.name);
    if(bone_name_index == (uintptr_t)-1){
      printf("f \"%s\"\n", bone.name.c_str());
    }
    else{
      auto solved_bone_name = get_model_bone_name(bone_name_index, model);
      if(solved_bone_name.size() == 0){
        printf("nu \"%s\" -> \"%s\"\n", bone.name.c_str(), bone_names_default.NA(bone_name_index)->sn);
      }
      else{
        printf("yay \"%s\" -> \"%s\" -> \"%s\"\n", bone.name.c_str(), bone_names_default.NA(bone_name_index)->sn, solved_bone_name.c_str());
      }
    }
  });
}

int main(){
  fan_3d::model::fms_t::properties_t p;
  p.path = "models/final_provence.fbx";
  p.texture_path = "models/textures";
  p.use_cpu = 0;
  fan_3d::model::fms_t model(p);
  p.path = "models/anim0.gltf";
  fan_3d::model::fms_t anim0(p);

  #if 0
  model.iterate_bones(*model.root_bone, [&](fan_3d::model::bone_t& bone) {
    printf("modelname %s\n", bone.name.c_str());
  });
  #endif

  mapper(model, anim0);

  return 0;
}
