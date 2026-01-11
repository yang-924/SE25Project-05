using System.Collections;
using System.Collections.Generic;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

/// <summary>
/// 自动为物体的所有子对象添加 MeshCollider 组件
/// 检测所有子对象（包括子对象的子对象），如果有 MeshRenderer 但没有 MeshCollider，则自动添加
/// 运行后会永久保存到场景中，只需要在编辑器中执行一次即可
/// </summary>
public class AutoAddMeshCollider : MonoBehaviour
{
    [Header("Settings")]
    [Tooltip("是否将 MeshCollider 设置为 Convex（凸面）")]
    public bool setConvex = false;

    [Tooltip("是否将 Collider 设置为 Trigger（仅触发事件，不产生物理碰撞）")]
    public bool isTrigger = false;

    private int addedCount = 0;

    /// <summary>
    /// 递归检查所有子对象,为有 MeshRenderer 但无 MeshCollider 的物体添加组件
    /// 此方法只能在编辑器中调用,会永久保存到场景文件中
    /// </summary>
    public void AddMeshCollidersToChildren()
    {
#if UNITY_EDITOR
        addedCount = 0;
        Debug.Log($"<color=cyan>[AutoAddMeshCollider]</color> 开始检测 {gameObject.name} 的所有子对象...");

        // 获取所有子对象（包括多层嵌套）
        MeshRenderer[] meshRenderers = GetComponentsInChildren<MeshRenderer>(true); // true 表示包含未激活的对象

        foreach (MeshRenderer renderer in meshRenderers)
        {
            GameObject obj = renderer.gameObject;

            // 检查是否已经有 MeshCollider
            MeshCollider existingCollider = obj.GetComponent<MeshCollider>();

            if (existingCollider == null)
            {
                // 没有 MeshCollider，添加一个
                MeshCollider newCollider = obj.AddComponent<MeshCollider>();
                newCollider.convex = setConvex;
                newCollider.isTrigger = isTrigger;

                // 标记对象为已修改，确保保存到场景中
                EditorUtility.SetDirty(obj);

                addedCount++;
                Debug.Log($"<color=green>[AutoAddMeshCollider]</color> 已为 {obj.name} 添加 MeshCollider (Convex: {setConvex}, Trigger: {isTrigger})");
            }
        }

        // 标记场景为已修改
        UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(gameObject.scene);

        Debug.Log($"<color=yellow>[AutoAddMeshCollider]</color> 完成！共添加了 {addedCount} 个 MeshCollider 组件。请记得保存场景 (Ctrl+S)。");
#else
        Debug.LogWarning("[AutoAddMeshCollider] 此方法只能在编辑器中使用！");
#endif
    }

    /// <summary>
    /// 移除所有自动添加的 MeshCollider（谨慎使用）
    /// </summary>
    public void RemoveAllMeshColliders()
    {
#if UNITY_EDITOR
        int removedCount = 0;
        MeshCollider[] colliders = GetComponentsInChildren<MeshCollider>(true);

        foreach (MeshCollider collider in colliders)
        {
            DestroyImmediate(collider);
            EditorUtility.SetDirty(collider.gameObject);
            removedCount++;
            Debug.Log($"<color=red>[AutoAddMeshCollider]</color> 已移除 {collider.gameObject.name} 的 MeshCollider");
        }

        // 标记场景为已修改
        UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(gameObject.scene);

        Debug.Log($"<color=yellow>[AutoAddMeshCollider]</color> 共移除了 {removedCount} 个 MeshCollider 组件。请记得保存场景 (Ctrl+S)。");
#else
        Debug.LogWarning("[AutoAddMeshCollider] 此方法只能在编辑器中使用！");
#endif
    }
}

#if UNITY_EDITOR
/// <summary>
/// 自定义 Inspector 面板，添加按钮
/// </summary>
[CustomEditor(typeof(AutoAddMeshCollider))]
public class AutoAddMeshColliderEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        AutoAddMeshCollider script = (AutoAddMeshCollider)target;

        EditorGUILayout.Space();
        EditorGUILayout.HelpBox("点击下方按钮在编辑器中添加 MeshCollider，会永久保存到场景中。", MessageType.Info);

        if (GUILayout.Button("添加 MeshCollider", GUILayout.Height(30)))
        {
            script.AddMeshCollidersToChildren();
        }

        EditorGUILayout.Space();

        if (GUILayout.Button("移除所有 MeshCollider", GUILayout.Height(30)))
        {
            if (EditorUtility.DisplayDialog("确认移除", "确定要移除所有子对象的 MeshCollider 吗？", "确定", "取消"))
            {
                script.RemoveAllMeshColliders();
            }
        }
    }
}
#endif
